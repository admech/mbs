within MbsLite.Examples.Misc;

model SingleBodyRolling

  parameter String      name    = "NOT INITIALIZED";
  parameter Real        m       = inf;
  parameter Real[3, 3]  I       (each start = inf);
  parameter Real        R       = inf;
  parameter Real[3]     v0      (each start = inf);
  parameter Real[3]     omega0  (each start = inf);

  Base base;

  NPortsHeavyBody body
    ( name = "body"
    , m = m
    , I = I
    , N = 1
    , Gravity = { 0, -1, 0 }
    , r(start = { 0, R, 0 })
    , v(start = v0)
    , q(start = { 1, 0, 0, 0 })
    , omega(start = omega0)
    );
  
  SpherePlaneContact contact
    ( name = "contact"
    , R = R
    );

initial algorithm
  AssertInitializedS(name, name,         "name");
  AssertInitialized (name, { m },        "m");
  AssertInitialized (name, I[1, :],      "I[1, :]");
  AssertInitialized (name, I[2, :],      "I[2, :]");
  AssertInitialized (name, I[3, :],      "I[3, :]");
  AssertInitialized (name, { R },        "R");
  AssertInitialized (name, v0,           "v0");
  AssertInitialized (name, omega0,       "omega0");

equation
  
  connect(body.OutPort,    contact.InPortA);
  connect(base.OutPort,    contact.InPortB);
  connect(body.InPorts[1], contact.OutPortA);

  assert(CompareReal(body.OutPort.r[2], R), "The ball fell down!");

end SingleBodyRolling;

