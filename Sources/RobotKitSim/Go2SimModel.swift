// Sources/RobotKitSim/Go2SimModel.swift
import Foundation

/// Locating and preparing the Menagerie Go2 model.
///
/// `go2.xml` declares no `<sensor>` block — only an unused `imu` site — so the
/// simulator loads a wrapper that includes the stock scene and adds the three
/// sensors needed to synthesize `unitree_go/IMUState`. Wrapping rather than
/// editing keeps the vendored model pristine.
public enum Go2SimModel {
    public static func wrapperXML(scenePath: String) -> String {
        """
        <mujoco model="go2_robotkit">
          <include file="\(scenePath)"/>
          <sensor>
            <gyro site="imu" name="rk_gyro"/>
            <accelerometer site="imu" name="rk_accel"/>
            <framequat objtype="site" objname="imu" name="rk_quat"/>
          </sensor>
        </mujoco>
        """
    }

    /// Finds `unitree_go2/scene.xml` under any of `searchDirs`.
    public static func resolveScene(searchDirs: [String]) -> String? {
        let fm = FileManager.default
        for root in searchDirs {
            let candidate = (root as NSString)
                .appendingPathComponent("unitree_go2/scene.xml")
            if fm.fileExists(atPath: candidate) { return candidate }
        }
        return nil
    }

    /// Writes the wrapper next to the scene so MuJoCo's `<include>` and the
    /// model's relative `meshdir` both resolve, and returns its path.
    public static func writeWrapper(besideScene scenePath: String) throws -> String {
        let dir = (scenePath as NSString).deletingLastPathComponent
        let wrapper = (dir as NSString).appendingPathComponent("robotkit_go2.xml")
        let xml = wrapperXML(scenePath: (scenePath as NSString).lastPathComponent)
        try xml.write(toFile: wrapper, atomically: true, encoding: .utf8)
        return wrapper
    }
}
