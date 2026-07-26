#ifndef WMJ_GL_H
#define WMJ_GL_H

#ifdef __cplusplus
extern "C" {
#endif

/// Opaque handle to a headless GL context suitable for MuJoCo's mjr_* renderer.
typedef struct wmj_gl_context wmj_gl_context;

/// Create a surfaceless GL context. Returns NULL on failure; call
/// wmj_gl_last_error() for a human-readable reason.
wmj_gl_context *wmj_gl_create(void);

/// Make ctx current on the calling thread. Returns 1 on success, 0 on failure.
int wmj_gl_make_current(wmj_gl_context *ctx);

/// Destroy ctx. Safe to call with NULL.
void wmj_gl_destroy(wmj_gl_context *ctx);

/// Last error message. Never NULL; empty string when there has been no error.
const char *wmj_gl_last_error(void);

/// "egl", "cgl", or "none" — which backend was compiled in.
const char *wmj_gl_backend_name(void);

#ifdef __cplusplus
}
#endif
#endif
