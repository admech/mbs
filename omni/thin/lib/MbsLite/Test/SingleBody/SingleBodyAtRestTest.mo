within MbsLite.Test.SingleBody;

model SingleBodyAtRestTest

  NPortsHeavyBody b
    ( name = "body"
    , m = 1
    , I = identity(3)
    , N = 0
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
      ( CompareReal(b.r * b.r, 0)
      , "b.r should be zero, was: " + StringA(b.r)
      );
  end when;
end SingleBodyAtRestTest;

