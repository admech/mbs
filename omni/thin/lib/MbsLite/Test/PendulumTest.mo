within MbsLite.Test;

model PendulumTest

  import MbsLite.Examples.Misc.Pendulum;

  parameter Real expectedAngle = -0.111270;

  Pendulum p
    ( name = "Pendulum"
    , m = 1
    , I = identity(3)
    , l = 1
    , angle0 = pi / 6
    );

  annotation(experiment
    ( StopTime = 10
    , NumberOfIntervals = 100
    , Tolerance = 1e-9
    , Algorithm = "Dassl"
    ));
equation
  when time == 10.0 then
    assert
      ( MbsLite.CompareReal(expectedAngle, p.joint.angle)
      , "angle should be " + String(expectedAngle) + ", was: " + String(p.joint.angle)
      );
  end when;
end PendulumTest;

