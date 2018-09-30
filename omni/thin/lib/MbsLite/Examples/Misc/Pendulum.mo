within MbsLite.Examples.Misc;

model Pendulum

  /*
     __________
        | \
        |  \
        |   \
        |    \
        |     \
        |      *
        |       `
        | angle0 `
  */

  parameter String     name;
  
  parameter Real       m;
  parameter Real[3, 3] I;
  parameter Real       l;
  parameter Real       angle0;

  parameter Real[4]    q0 = QRot(angle0, { 0, 0, 1 });
  parameter Real[3]    bodyAxisPositionLocal = { 0, l, 0 };
  parameter Real[3]    bodyMassCenterGlobal = QToT(q0) * (-bodyAxisPositionLocal);
  parameter Real[3]    userwardNormalToTheScreen = { 0, 0, 1 };

  Real[3] F;
  Real[3] M;

  Base base;

  NPortsHeavyBody body
    ( name = name + ".body"
    , N = 1
    , Gravity = { 0, -1, 0 }
    , m = m
    , I = I
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

initial algorithm
  AssertInitializedS(name, name,       "name");
  AssertInitialized(name,  { m },      "m");
  AssertInitialized(name,  I[1, :],    "I[1, :]");
  AssertInitialized(name,  I[2, :],    "I[2, :]");
  AssertInitialized(name,  I[3, :],    "I[3, :]");
  AssertInitialized(name,  { l },      "l");
  AssertInitialized(name,  { angle0 }, "angle0");

equation

  connect(base.OutPort, joint.InPortA);
  joint.OutPortA.F = F;
  joint.OutPortA.M = M;

  connect(body.OutPort,    joint.InPortB);
  connect(body.InPorts[1], joint.OutPortB);

  assert
    ( CompareReal(1, norm(body.r))
    , "body should stay at distance of 1 from origin, was: " + String(norm(body.r))
    );

  assert
    ( CompareReal(0, norm(joint.RA), absTol = 1e-8)
    , "joint should be fixed at origin, was: " + StringA(joint.RA)
    );

  assert
    ( CompareReal(0, norm(joint.RB), absTol = 1e-8)
    , "joint fixture point in body should coincide with origin, was: " + StringA(joint.RB)
    );

end Pendulum;


