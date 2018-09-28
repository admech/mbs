within MbsLite.Test;

model SingleBodyAtRestTest

  MbsLite.NPortsHeavyBody b
    ( N = 0
    , Gravity = { 0, 0, 0 }
    , r(start = { 0, 0, 0 })
    , v(start = { 0, 0, 0 })
    , q(start = { 1, 0, 0, 0 })
    , omega(start = { 0, 0, 0 })
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
      ( MbsLite.CompareReal(b.r * b.r, 0)
      , "b.r should be zero, was: " + MbsLite.StringA(b.r)
      );
  end when;
end SingleBodyAtRestTest;

