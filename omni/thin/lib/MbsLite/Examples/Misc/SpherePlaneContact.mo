within MbsLite.Examples.Misc;

model SpherePlaneContact
  "A -- sphere, B -- plane"
  extends Constraint
  ;

  parameter String  name      = "NOT INITIALIZED";
  parameter Real    R         = inf;

  constant Real[3] vertical  = { 0, 1, 0 };

  Real[3] rc "coordinates of the point of contact in global coords";
  Real[3] vA "velocity of the contacting point of the ball";
  Real[3] vB "velocity of the contacting point of the plane";
  Real[3] relV;
  Real[3] DrelV;
  Real    relVN;
  Real[3] relVT;

initial algorithm
  AssertInitializedS(name, name,  "name");
  AssertInitialized (name, { R }, "R");

equation

  // contact point is always exactly under the center of the ball
  rc = InPortA.r - R * vertical;

  vA = Euler(InPortA.r, rc, InPortA.v, InPortA.omega);
  vB = Euler(InPortB.r, rc, InPortB.v, InPortB.omega);
  relV = vB - vA;
  relVN = relV * vertical;
  relVT = relV - relVN * vertical;

  // below conditions are applied to the acceleration for numerical tractability

  // make the ball keep itself on top of the plane and not fall through
  assert(CompareReal(relVN, 0), "the ball is falling down!");

  // disallow slippage
  assert(CompareReal(norm(relVT), 0), "the ball is slipping!");

  der(relV) = DrelV;
  DrelV = zeros(3);

  // "export" force information

  OutPortA.P = rc;
  OutPortB.P = rc;

  // no torque in contact
  OutPortA.M = zeros(3);

  // force is actually not being exported because it's supposed
  // to be calculated by Signorini's law
  // via the requirement for velocities to be zero
  
end SpherePlaneContact;

