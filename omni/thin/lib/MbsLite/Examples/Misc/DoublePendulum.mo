within MbsLite.Examples.Misc;

model DoublePendulum

  /*
     __________
        | \
        |  \
        |   \
        |    \
        |     \
        |      *
        |       `
        | angle10`
                  | \
                  |  \
                  |   \
                  |    \
                  |     \
                  |      *
                  |       `
                  | angle20`
  */

  parameter String     name;
  
  parameter Real       m;
  parameter Real[3, 3] I;
  parameter Real       l;
  parameter Real       angle10;
  parameter Real       angle20;

  parameter Real[4]    q10 = QRot(angle10, { 0, 0, 1 });
  parameter Real[4]    q20 = QRot(angle20, { 0, 0, 1 });
  parameter Real[3]    bodyAxisPositionLocal = { 0, l, 0 };
  parameter Real[3]    body1MassCenterGlobal = QToT(q10) * (-bodyAxisPositionLocal);
  parameter Real[3]    body2MassCenterGlobal = body1MassCenterGlobal + QToT(q20) * (-bodyAxisPositionLocal);
  parameter Real[3]    userwardNormalToTheScreen = { 0, 0, 1 };

  Base base;

  NPortsHeavyBody body1
    ( name = name + ".body1"
    , N = 1
    , Gravity = { 0, -1, 0 }
    , m = m
    , I = I
    , r(start = body1MassCenterGlobal)
    , v(start = { 0, 0, 0 })
    , q(start = q10)
    , omega(start = { 0, 0, 0 })
    );

  NPortsHeavyBody body2
    ( name = name + ".body2"
    , N = 1
    , Gravity = { 0, -1, 0 }
    , m = m
    , I = I
    , r(start = body2MassCenterGlobal)
    , v(start = { 0, 0, 0 })
    , q(start = q20)
    , omega(start = { 0, 0, 0 })
    );

  FixedJoint joint1 // A -- base, B -- body1
    ( name = "joint1"
    , nA = userwardNormalToTheScreen
    , nB = userwardNormalToTheScreen
    , rA = { 0, 0, 0 }
    , rB = bodyAxisPositionLocal
    );

  FixedJoint joint2 // A -- body1, B -- body2
    ( name = "joint2"
    , nA = userwardNormalToTheScreen
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
  AssertInitialized(name,  { angle10 }, "angle10");
  AssertInitialized(name,  { angle20 }, "angle20");

equation

  connect(base.OutPort,     joint1.InPortA);
  connect(body1.OutPort,    joint1.InPortB);
  connect(body1.InPorts[1], joint1.OutPortB);

  connect(body1.OutPort,    joint2.InPortA);
  connect(body2.OutPort,    joint2.InPortB);
  connect(body2.InPorts[1], joint2.OutPortB);

  assert
    ( CompareReal(1, norm(body1.r))
    , "body1 should stay at distance of 1 from origin, was: " + String(norm(body1.r))
    );

  assert
    ( CompareReal(1, norm(body2.r - body1.r))
    , "body2 should stay at distance of 1 from body1, was: " + String(norm(body2.r - body1.r))
    );

end DoublePendulum;


