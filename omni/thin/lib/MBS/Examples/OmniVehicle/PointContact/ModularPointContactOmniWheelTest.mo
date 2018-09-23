within MBS.Examples.OmniVehicle.PointContact;

model ModularPointContactOmniWheelTest

  parameter Integer NRollers = 4 "Number of rollers";
  parameter Real alpha = Modelica.Constants.pi/NRollers
    "Max angle of the half-sector";
  parameter Real R = 1 "Omni wheel outer radius";
  parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
  parameter Real[3] v0 = {1, 0, 0};
  parameter Real[3] omega0 = {0, 1, -1};

  ForceBase floor;
  OmniWheel wheel(
    alpha = alpha,
    R = R,
    R1 = R1,
    v0 = v0,
    omega0 = omega0);

  // for plotting:
  Real[3] _r;
  Real[3] _v;
  Real _omega;
  Real _angle(start = 0);

equation
  connect(wheel.InPortK, floor.OutPort);
  connect(wheel.InPortF, floor.InPort);

  _r = wheel.Wheel.r;
  _v = wheel.Wheel.v;
  _omega = wheel.Wheel.omega[2];
  der(_angle) = _omega;

end ModularPointContactOmniWheelTest;
