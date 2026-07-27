#include "include/wmj_gl.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char g_err[512] = {0};

static void set_err(const char *fmt, const char *detail) {
    snprintf(g_err, sizeof(g_err), fmt, detail ? detail : "");
}

const char *wmj_gl_last_error(void) { return g_err; }

/* ------------------------------------------------------------------ Linux: EGL */
#if defined(__linux__)

#include <dlfcn.h>

/* Minimal EGL surface. We declare the handful of types and constants we need
   rather than including <EGL/egl.h>, so this target has no build-time
   dependency on EGL headers being installed. */
typedef void *EGLDisplay;
typedef void *EGLContext;
typedef void *EGLConfig;
typedef void *EGLSurface;
typedef int   EGLint;
typedef unsigned int EGLBoolean;
typedef unsigned int EGLenum;

#define EGL_DEFAULT_DISPLAY   ((void *)0)
#define EGL_NO_CONTEXT        ((EGLContext)0)
#define EGL_NO_SURFACE        ((EGLSurface)0)
#define EGL_NONE              0x3038
#define EGL_SURFACE_TYPE      0x3033
#define EGL_PBUFFER_BIT       0x0001
#define EGL_RED_SIZE          0x3024
#define EGL_GREEN_SIZE        0x3023
#define EGL_BLUE_SIZE         0x3022
#define EGL_ALPHA_SIZE        0x3021
#define EGL_DEPTH_SIZE        0x3025
#define EGL_STENCIL_SIZE      0x3026
#define EGL_RENDERABLE_TYPE   0x3040
#define EGL_OPENGL_BIT        0x0008
#define EGL_OPENGL_API        0x30A2

typedef EGLDisplay (*fn_GetDisplay)(void *);
typedef EGLBoolean (*fn_Initialize)(EGLDisplay, EGLint *, EGLint *);
typedef EGLBoolean (*fn_ChooseConfig)(EGLDisplay, const EGLint *, EGLConfig *, EGLint, EGLint *);
typedef EGLBoolean (*fn_BindAPI)(EGLenum);
typedef EGLContext (*fn_CreateContext)(EGLDisplay, EGLConfig, EGLContext, const EGLint *);
typedef EGLBoolean (*fn_MakeCurrent)(EGLDisplay, EGLSurface, EGLSurface, EGLContext);
typedef EGLBoolean (*fn_DestroyContext)(EGLDisplay, EGLContext);
typedef EGLBoolean (*fn_Terminate)(EGLDisplay);

struct wmj_gl_context {
    void *lib;
    EGLDisplay dpy;
    EGLContext ctx;
    fn_MakeCurrent MakeCurrent;
    fn_DestroyContext DestroyContext;
    fn_Terminate Terminate;
};

const char *wmj_gl_backend_name(void) { return "egl"; }

wmj_gl_context *wmj_gl_create(void) {
    g_err[0] = 0;

    void *lib = dlopen("libEGL.so.1", RTLD_LAZY | RTLD_LOCAL);
    if (!lib) lib = dlopen("libEGL.so", RTLD_LAZY | RTLD_LOCAL);
    if (!lib) {
        set_err("cannot load libEGL.so.1 (%s) - install libegl1 and, for CPU-only "
                "rendering, libgl1-mesa-dri", dlerror());
        return NULL;
    }

#define LOAD(var, type, name)                                                  \
    type var = (type)dlsym(lib, name);                                         \
    if (!var) { set_err("libEGL is missing symbol %s", name); dlclose(lib); return NULL; }

    LOAD(GetDisplay, fn_GetDisplay, "eglGetDisplay")
    LOAD(Initialize, fn_Initialize, "eglInitialize")
    LOAD(ChooseConfig, fn_ChooseConfig, "eglChooseConfig")
    LOAD(BindAPI, fn_BindAPI, "eglBindAPI")
    LOAD(CreateContext, fn_CreateContext, "eglCreateContext")
    LOAD(MakeCurrent, fn_MakeCurrent, "eglMakeCurrent")
    LOAD(DestroyContext, fn_DestroyContext, "eglDestroyContext")
    LOAD(Terminate, fn_Terminate, "eglTerminate")
#undef LOAD

    EGLDisplay dpy = GetDisplay(EGL_DEFAULT_DISPLAY);
    if (!dpy) {
        set_err("eglGetDisplay returned no display%s", "");
        dlclose(lib);
        return NULL;
    }
    EGLint major = 0, minor = 0;
    if (!Initialize(dpy, &major, &minor)) {
        set_err("eglInitialize failed - no usable EGL driver%s", "");
        dlclose(lib);
        return NULL;
    }

    const EGLint cfg_attribs[] = {
        EGL_SURFACE_TYPE,    EGL_PBUFFER_BIT,
        EGL_RED_SIZE,        8,
        EGL_GREEN_SIZE,      8,
        EGL_BLUE_SIZE,       8,
        EGL_ALPHA_SIZE,      8,
        EGL_DEPTH_SIZE,      24,
        EGL_STENCIL_SIZE,    8,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
        EGL_NONE
    };
    EGLConfig cfg;
    EGLint n = 0;
    if (!ChooseConfig(dpy, cfg_attribs, &cfg, 1, &n) || n < 1) {
        set_err("eglChooseConfig found no desktop-OpenGL config%s", "");
        Terminate(dpy);
        dlclose(lib);
        return NULL;
    }

    /* MuJoCo's mjr_* renderer needs desktop OpenGL, not GLES. */
    if (!BindAPI(EGL_OPENGL_API)) {
        set_err("eglBindAPI(EGL_OPENGL_API) failed - driver offers only GLES%s", "");
        Terminate(dpy);
        dlclose(lib);
        return NULL;
    }

    EGLContext ctx = CreateContext(dpy, cfg, EGL_NO_CONTEXT, NULL);
    if (!ctx) {
        set_err("eglCreateContext failed%s", "");
        Terminate(dpy);
        dlclose(lib);
        return NULL;
    }

    wmj_gl_context *out = (wmj_gl_context *)calloc(1, sizeof(wmj_gl_context));
    if (!out) {
        set_err("out of memory%s", "");
        DestroyContext(dpy, ctx);
        Terminate(dpy);
        dlclose(lib);
        return NULL;
    }
    out->lib = lib;
    out->dpy = dpy;
    out->ctx = ctx;
    out->MakeCurrent = MakeCurrent;
    out->DestroyContext = DestroyContext;
    out->Terminate = Terminate;
    return out;
}

int wmj_gl_make_current(wmj_gl_context *ctx) {
    if (!ctx) return 0;
    if (!ctx->MakeCurrent(ctx->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx->ctx)) {
        set_err("eglMakeCurrent failed%s", "");
        return 0;
    }
    return 1;
}

void wmj_gl_destroy(wmj_gl_context *ctx) {
    if (!ctx) return;
    ctx->MakeCurrent(ctx->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    ctx->DestroyContext(ctx->dpy, ctx->ctx);
    /* Deliberately do NOT call ctx->Terminate(ctx->dpy) or dlclose(ctx->lib)
       here. wmj_gl_create always fetches the display via
       eglGetDisplay(EGL_DEFAULT_DISPLAY); per EGL 1.5 SS3.2, repeated calls with
       the same display_id return the SAME EGLDisplay handle, and
       eglInitialize/eglTerminate are not reference-counted. eglTerminate
       destroys every resource on that display not current to some thread —
       so terminating here would destroy any OTHER live wmj_gl_context sharing
       this process's default display (two MjOffscreenRenderers, or the
       isAvailable probe running next to a live renderer, or the test suite
       running contexts in parallel). dlclose(ctx->lib) is similarly unsafe:
       a concurrent wmj_gl_create can be inside libEGL on another thread while
       this thread unloads it, and Mesa/glvnd are not reliably safe to unload
       and reload.
       Both the display and the libEGL.so handle are process-global
       singletons, so we leak them on purpose — the same choice established
       headless-EGL backends (e.g. dm_control's) make. Do NOT "fix" this back;
       see the CRITICAL finding in the 0.2.0 final review for the failure
       mode this avoids. The failure paths inside wmj_gl_create are unaffected
       and still correctly call Terminate/dlclose there, since no context was
       ever successfully created in those cases. */
    free(ctx);
}

/* -------------------------------------------------------------- macOS: CGL */
#elif defined(__APPLE__)

/* Apple deprecated all of OpenGL in favour of Metal, but MuJoCo's classic mjr_*
   renderer requires a desktop GL context and CGL is the only way to create one
   on macOS. The deprecation is acknowledged and deliberate — silence it rather
   than carry 6 warnings on every clean build. */
#define GL_SILENCE_DEPRECATION 1
#include <OpenGL/OpenGL.h>

struct wmj_gl_context { CGLContextObj ctx; };

const char *wmj_gl_backend_name(void) { return "cgl"; }

wmj_gl_context *wmj_gl_create(void) {
    g_err[0] = 0;
    CGLPixelFormatAttribute attribs[] = {
        kCGLPFAOpenGLProfile, (CGLPixelFormatAttribute)kCGLOGLPVersion_Legacy,
        kCGLPFAColorSize,     (CGLPixelFormatAttribute)24,
        kCGLPFAAlphaSize,     (CGLPixelFormatAttribute)8,
        kCGLPFADepthSize,     (CGLPixelFormatAttribute)24,
        kCGLPFAStencilSize,   (CGLPixelFormatAttribute)8,
        (CGLPixelFormatAttribute)0
    };
    CGLPixelFormatObj pix = NULL;
    GLint npix = 0;
    if (CGLChoosePixelFormat(attribs, &pix, &npix) != kCGLNoError || !pix) {
        set_err("CGLChoosePixelFormat found no offscreen-capable format%s", "");
        return NULL;
    }
    CGLContextObj cgl = NULL;
    CGLError e = CGLCreateContext(pix, NULL, &cgl);
    CGLDestroyPixelFormat(pix);
    if (e != kCGLNoError || !cgl) {
        set_err("CGLCreateContext failed: %s", CGLErrorString(e));
        return NULL;
    }
    wmj_gl_context *out = (wmj_gl_context *)calloc(1, sizeof(wmj_gl_context));
    if (!out) { set_err("out of memory%s", ""); CGLDestroyContext(cgl); return NULL; }
    out->ctx = cgl;
    return out;
}

int wmj_gl_make_current(wmj_gl_context *ctx) {
    if (!ctx) return 0;
    if (CGLSetCurrentContext(ctx->ctx) != kCGLNoError) {
        set_err("CGLSetCurrentContext failed%s", "");
        return 0;
    }
    return 1;
}

void wmj_gl_destroy(wmj_gl_context *ctx) {
    if (!ctx) return;
    CGLSetCurrentContext(NULL);
    CGLDestroyContext(ctx->ctx);
    free(ctx);
}

/* ------------------------------------------------------- everything else */
#else

struct wmj_gl_context { int unused; };

const char *wmj_gl_backend_name(void) { return "none"; }

wmj_gl_context *wmj_gl_create(void) {
    set_err("no GL backend compiled for this platform%s", "");
    return NULL;
}
int wmj_gl_make_current(wmj_gl_context *ctx) { (void)ctx; return 0; }
void wmj_gl_destroy(wmj_gl_context *ctx) { (void)ctx; }

#endif
