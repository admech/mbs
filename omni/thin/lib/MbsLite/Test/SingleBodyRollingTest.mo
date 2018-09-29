within MbsLite.Test;

model SingleBodyRollingTest

  import MbsLite.Examples.Misc.SingleBodyRolling;

  parameter Real R  = 1;
  parameter Real v0 = 1;
  parameter Real stopTime = 10;

  parameter Real[3] vertical   = { 0, 1, 0 };
  parameter Real[3] v0vec      = QToT(QRot( -pi / 6, vertical )) * { v0, 0, 0 };
  parameter Real[3] omega0     = 1 / R * cross(vertical, v0vec) + { 0, 2, 0 }; // vertical spinning does not matter, is just preserved
  parameter Real[3] rExpected  = R * vertical + stopTime * v0vec;

  SingleBodyRolling m
    ( name = "a ball rolling on a plane"
    , m = 1
    , I = identity(3)
    , R = 1
    , v0     = v0vec
    , omega0 = omega0
    );

  annotation(experiment
    ( StopTime = 10
    , NumberOfIntervals = 1000
    , Tolerance = 1e-9
    , Algorithm = "Dassl"
    ));
equation

  when time == stopTime then
    for i in 1 : 3 loop
      assert(CompareReal(rExpected[i], m.body.OutPort.r[i]), "the ball should have gone to r = v0 * t, at " + String(i) + " = " + String(rExpected[i]) + ", but coord " + String(i) + " was: " + String(m.body.OutPort.r[i]));
      assert(CompareReal(omega0[i], m.body.OutPort.omega[i]), "angular velocity should be constant, component " + String(i) + " should have been: " + String(omega0[i]) + ", but was: " + String(m.body.OutPort.omega[i]));
    end for;
  end when;

end SingleBodyRollingTest;

