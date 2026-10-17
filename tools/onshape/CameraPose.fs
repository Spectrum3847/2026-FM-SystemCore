FeatureScript 2625;
import(path : "onshape/std/common.fs", version : "2625.0");

// Camera Pose: each camera lens's pose relative to the robot center on the floor, as a WPILib
// Transform3d and as Limelight web UI values.
//
// Robot center mate connector: on the floor under the robot's center, X forward, Z up.
// Lens mate connector: at the lens center, Z out of the lens along the optical axis, Y toward the
// top of the image.
//
// Results go to the Variables table (<name>_x, _y, _z in meters; _roll, _pitch, _yaw in degrees,
// WPILib convention), the FeatureScript console (copy-paste Java and Limelight lines) and the
// feature's info message.
//
// Limelight UI: forward, right, up; pitch positive up; yaw positive counter-clockwise seen
// from above, as in Vision's Limelight configs.

annotation { "Feature Type Name" : "Camera Pose",
        "Feature Type Description" : "Camera lens poses relative to the robot center on the floor, as WPILib Transform3d and Limelight UI values." }
export const cameraPose = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Robot center on floor (X forward, Z up)", "Filter" : BodyType.MATE_CONNECTOR, "MaxNumberOfPicks" : 1 }
        definition.robotCenter is Query;

        annotation { "Name" : "Cameras", "Item name" : "Camera", "Item label template" : "#name" }
        definition.cameras is array;
        for (var camera in definition.cameras)
        {
            annotation { "Name" : "Name (letters, digits, _)", "Default" : "camera" }
            camera.name is string;

            annotation { "Name" : "Lens (Z out of lens, Y image up)", "Filter" : BodyType.MATE_CONNECTOR, "MaxNumberOfPicks" : 1 }
            camera.lens is Query;
        }
    }
    {
        if (isQueryEmpty(context, definition.robotCenter))
            throw regenError("Pick the robot center mate connector.", ["robotCenter"]);

        const robot = evMateConnector(context, { "mateConnector" : definition.robotCenter });
        const rx = robot.xAxis;
        const rz = robot.zAxis;
        const ry = cross(rz, rx);

        var info = "";
        for (var i = 0; i < size(definition.cameras); i += 1)
        {
            const camera = definition.cameras[i];
            if (match(camera.name, "[A-Za-z_][A-Za-z0-9_]*").hasMatch == false)
                throw regenError("Camera names may use only letters, digits and _ and cannot start with a digit.");
            if (isQueryEmpty(context, camera.lens))
                throw regenError("Pick the lens mate connector for " ~ camera.name ~ ".");

            const lens = evMateConnector(context, { "mateConnector" : camera.lens });
            // WPILib camera frame: forward along the optical axis, left, up.
            const fwd = lens.zAxis;
            const up = cross(lens.zAxis, lens.xAxis);
            const left = cross(up, fwd);

            const d = lens.origin - robot.origin;
            const x = dot(d, rx);
            const y = dot(d, ry);
            const z = dot(d, rz);

            // Rotation3d(roll, pitch, yaw) is extrinsic X, then Y, then Z.
            const r00 = dot(rx, fwd);
            const r10 = dot(ry, fwd);
            const r20 = dot(rz, fwd);
            const r21 = dot(rz, left);
            const r22 = dot(rz, up);
            const yaw = atan2(r10, r00);
            const pitch = asin(clamp(-r20, -1, 1));
            const roll = atan2(r21, r22);

            const n = camera.name;
            setVariable(context, n ~ "_x", x);
            setVariable(context, n ~ "_y", y);
            setVariable(context, n ~ "_z", z);
            setVariable(context, n ~ "_roll", roll);
            setVariable(context, n ~ "_pitch", pitch);
            setVariable(context, n ~ "_yaw", yaw);

            const xm = fmt(x / meter);
            const ym = fmt(y / meter);
            const zm = fmt(z / meter);
            const rollD = fmt(roll / degree);
            const pitchD = fmt(pitch / degree);
            const yawD = fmt(yaw / degree);

            println(n ~ " WPILib: new Transform3d(new Translation3d(" ~ xm ~ ", " ~ ym ~ ", " ~ zm ~ "), new Rotation3d(Units.degreesToRadians(" ~ rollD ~ "), Units.degreesToRadians(" ~ pitchD ~ "), Units.degreesToRadians(" ~ yawD ~ ")))");
            println(n ~ " Limelight UI: forward " ~ xm ~ " m, right " ~ fmt(-y / meter) ~ " m, up " ~ zm ~ " m, roll " ~ rollD ~ ", pitch " ~ fmt(-pitch / degree) ~ ", yaw " ~ yawD);
            if (abs(pitch / degree) > 85)
                println(n ~ ": pitch is near 90 deg, so roll and yaw are not well defined.");

            info = info ~ n ~ ": " ~ xm ~ ", " ~ ym ~ ", " ~ zm ~ " m, r/p/y " ~ rollD ~ ", " ~ pitchD ~ ", " ~ yawD ~ "   ";
        }
        if (info != "")
            reportFeatureInfo(context, id, info);
    });

function fmt(value is number) returns string
{
    return toString(roundToPrecision(value, 4));
}
