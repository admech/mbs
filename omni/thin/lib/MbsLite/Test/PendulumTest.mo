within MbsLite.Test;

model PendulumTest

  import MbsLite.Examples.Misc.Pendulum;

  Pendulum p
    ( name = "Pendulum"
    , m = 1
    , I = identity(3)
    , l = 1
    , angle0 = pi / 6
    );

  annotation(experiment
    ( StopTime = 0.011
    , NumberOfIntervals = 100
    , Tolerance = 1e-9
    , Algorithm = "Dassl"
    ));
equation
  when time > 0.01 then
    assert
      ( MbsLite.CompareReal(0, 0)
      , "zero should be zero, was: " + String(0)
      );
  end when;
end PendulumTest;

