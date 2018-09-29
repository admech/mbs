within MbsLite.Test;

model SingleBodyRollingTest

  import MbsLite.Examples.Misc.SingleBodyRolling;

  SingleBodyRolling modelUnderTest
    ( name = "a ball rolling on a plane"
    , m = 1
    , I = identity(3)
    , R = 1
    , v0     = { 0, 0, 0 }
    , omega0 = { 0, 0, 0 }
    );

  annotation(experiment
    ( StopTime = 0.011
    , NumberOfIntervals = 100
    , Tolerance = 1e-9
    , Algorithm = "Dassl"
    ));
equation
  when time > 0.01 then
    print("foo");
  end when;
end SingleBodyRollingTest;

