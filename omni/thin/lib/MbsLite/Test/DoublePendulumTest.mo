within MbsLite.Test;

model DoublePendulumTest

  import MbsLite.Examples.Misc.DoublePendulum;

  parameter Real expectedAngle1 = -0.0380723;
  parameter Real expectedAngle2 =  12.1805;

  DoublePendulum p
    ( name = "DoublePendulum"
    , m = 1
    , I = identity(3)
    , l = 1
    , angle10 = pi / 6
    , angle20 = pi / 2
    );

  annotation(experiment
    ( StopTime = 100
    , NumberOfIntervals = 1000
    , Tolerance = 1e-9
    , Algorithm = "Dassl"
    ));
equation
  when time == 100.0 then
    assert
      ( MbsLite.CompareReal(expectedAngle1, p.joint1.angle)
      , "angle1 should be " + String(expectedAngle1) + ", was: " + String(p.joint1.angle)
      );
    assert
      ( MbsLite.CompareReal(expectedAngle2, p.joint2.angle)
      , "angle2 should be " + String(expectedAngle2) + ", was: " + String(p.joint2.angle)
      );
  end when;
end DoublePendulumTest;
