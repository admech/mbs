within MbsLite.Test.SingleBody;

model SingleBodyFallingTest

  NPortsHeavyBody b
    ( name = "body"
    , m = 1
    , I = identity(3)
    , N = 0
    , Gravity = { 0, -1, 0 }
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
      ( CompareReal(b.r[1], 0)
      , "b.r[1] should be zero, was: " + String(b.r[1])
      );
    assert
      ( CompareReal(b.r[2], -5e-5)
      , "b.r[2] should be -5e-5, was: " + String(b.r[2])
      );
    assert
      ( CompareReal(b.r[3], 0)
      , "b.r[3] should be zero, was: " + String(b.r[3])
      );
  end when;
end SingleBodyFallingTest;

