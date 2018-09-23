within MBS.Examples.OmniVehicle.PointContact;

model NRollersPointContactOmniWheelGeneralVerticalStepTest
  import Modelica.Constants.pi;

  //  parameter Real _psi = 0;
  parameter Real _psi = 10e-1;//pi/4;
  //  parameter Real _psi = 7e-1;

  parameter Integer NRollers = 4 "Number of rollers";
  parameter Real alpha = Modelica.Constants.pi/NRollers
    "Max angle of the half-sector";
  parameter Real R = 1 "Omni wheel outer radius";
  parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
  parameter Real initial_spin = 0; //needs checking
  parameter Real[3] omega0 = -1 * {sin(initial_spin), 0, cos(initial_spin)} * 2*alpha;
  parameter Real[4] _q0 = {cos(initial_spin/2), 0, sin(initial_spin/2), 0};
  parameter Real[3] v0 = cross(R*{0, -1, 0}, omega0);
  parameter Real[3] nA = {0, 1, 0} "Vertical";

  //  ForceBase floor;
  Base floor;
  NRollersOmniWheelGeneralStep wheel(
    psi = _psi,
    NRollers = NRollers,
    alpha = alpha,
    R = R,
    R1 = R1,
    v0 = v0,
    q0 = _q0,
    omega0 = omega0);

  Real[3] k1;
  Real[3] j1;
  Real[3] i2;
  Real[3] i1;

  // for plotting:
  Real[3] _r;
  Real[3] _v;
  Real _omega(start = omega0[3]);
  Real _angle(start = 0);
  Real lambda(start = 0);
equation
  wheel.Contacts[1].n1k = {wheel.OutPortK.T[1,3], wheel.OutPortK.T[2,3], wheel.OutPortK.T[3,3]};
  wheel.Contacts[2].n1k = {wheel.OutPortK.T[1,3], wheel.OutPortK.T[2,3], wheel.OutPortK.T[3,3]};
  wheel.Contacts[3].n1k = {wheel.OutPortK.T[1,3], wheel.OutPortK.T[2,3], wheel.OutPortK.T[3,3]};
  wheel.Contacts[4].n1k = {wheel.OutPortK.T[1,3], wheel.OutPortK.T[2,3], wheel.OutPortK.T[3,3]};
  wheel.Contacts[1].rO = wheel.Wheel.r;
  wheel.Contacts[2].rO = wheel.Wheel.r;
  wheel.Contacts[3].rO = wheel.Wheel.r;
  wheel.Contacts[4].rO = wheel.Wheel.r;

  connect(wheel.InPortK, floor.OutPort);

  wheel.OutPortK.T[2,3] = 0;

  k1 = {wheel.OutPortK.T[1,3], wheel.OutPortK.T[2,3], wheel.OutPortK.T[3,3]};
  j1 = nA;
  i2 = cross(j1, k1);
  i1 = i2/sqrt(i2*i2);
  wheel.InPortF.F = {0,0,0};
  wheel.InPortF.M = lambda*i1;
  //  wheel.InPortF.M = {lambda,0,0};
  wheel.InPortF.P = wheel.Wheel.r;

  _r = wheel.Wheel.r;

  _v = wheel.Wheel.v;
  _omega = wheel.Wheel.omega[3];
  der(_angle) = _omega * 180 / pi;

  annotation (experiment(
      StopTime=10,
      NumberOfIntervals=50000,
      Tolerance=1e-010,
      Algorithm="Dassl"),experimentSetupOutput);
end NRollersPointContactOmniWheelGeneralVerticalStepTest;
