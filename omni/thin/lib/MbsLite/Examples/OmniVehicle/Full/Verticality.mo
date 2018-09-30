within MbsLite.Examples.OmniVehicle.Full;

model Verticality
   "Zeroes the component of angular velocity along the 'forward' direction:
     forward = cross(vertical, axis)
   where axis, if the wheel is attached to a vehicle, looks outward, and
   the wheel tries to rotate the vehicle counter-clockwise (looking from top),
   if omegaRoll > 0."

  parameter String name = "NOT INITIALIZED";

  constant Real[3] forward    = { 1, 0, 0 };
  constant Real[3] vertical   = { 0, 1, 0 };
  constant Real[3] wheelAxis  = { 0, 0, 1 };

  KinematicPort  InPort;
  WrenchPort     OutPort;

  // kren, tangazh and ryskanie == roll, pitch and yaw; this is kren, i.e. roll
  Real omegaRoll;
  Real DomegaRoll;

initial algorithm
  AssertInitializedS(name, name, "name");

equation

  omegaRoll = InPort.omega * InPort.T * { 1, 0, 0 };
  der(omegaRoll) = DomegaRoll;
  DomegaRoll = 0;
  assert(CompareReal(omegaRoll, 0), "the wheel tipped!");

  OutPort.P = InPort.r;
  OutPort.F = zeros(3);
  // torque along the "forward" direction is not being specified,
  // but instead, calculated by Signorini's law;
  // the other two remain zero as the constraint is ideal
  0 = OutPort.M * InPort.T * forward;
  0 = OutPort.M * InPort.T * wheelAxis;

end Verticality;
