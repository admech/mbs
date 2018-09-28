within MbsLite.Examples.Misc;

model Pendulum

  /*
     __________
        | \
        |  \
        |   \
        |    *
        |     `
        | pi/6 `
  */

  Base base;

  parameter Real[4] q0 = QRot(pi/6, { 0, 0, 1 });
  parameter Real[3] bodyAxisPositionLocal = { 0, 1, 0 };
  parameter Real[3] bodyMassCenterGlobal = QToT(q0) * (-bodyAxisPositionLocal);
  parameter Real[3] userwardNormalToTheScreen = { 0, 0, 1 };

  NPortsHeavyBody body
    ( N = 1
    , Gravity = { 0, -1, 0 }
    , r(start = bodyMassCenterGlobal)
    , v(start = { 0, 0, 0 })
    , q(start = q0)
    , omega(start = { 0, 0, 0 })
    );

  FixedJoint joint // A -- base, B -- body
    ( nA = userwardNormalToTheScreen
    , nB = userwardNormalToTheScreen
    , rA = { 0, 0, 0 }
    , rB = bodyAxisPositionLocal
    );

equation

  connect(base.OutPort, joint.InPortA);
  connect(body.OutPort, joint.InPortB);
  connect(body.InPorts[1], joint.OutPortB);

  assert
    ( CompareReal(1, norm(body.r))
    , "body should stay at distance of 1 from origin, was: " + String(norm(body.r))
    );

end Pendulum;


