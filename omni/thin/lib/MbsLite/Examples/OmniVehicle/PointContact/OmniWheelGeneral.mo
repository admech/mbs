within MbsLite.Examples.OmniVehicle.PointContact;

model OmniWheelGeneral

  parameter Real pi = Modelica.Constants.pi;
  parameter SI.Acceleration[3] Gravity = { 0, -1, 0 };

  parameter Integer      n        = 4               "Number of rollers";
  parameter Real         alpha    = pi / n          "Max angle of the half-sector";
  parameter Real         R        = 1               "Omni wheel outer radius";
  parameter Real         R1       = R * cos(alpha)  "Omni wheel inner radius";
  parameter Real[3]      r0       = { 0, R, 0 };
  parameter Real[4]      q0       = { 1, 0, 0, 0 };
  parameter Real[3]      v0       = { 1, 0, 0 };
  parameter Real[3]      omega0   = { 0, 0, -1 };
  parameter Real[3, 3]   T0       = identity(3);

  annotation
  ( experiment
      ( StopTime=40
      , NumberOfIntervals=50000
      , Tolerance=1e-008
      )
  , experimentSetupOutput
  );

  parameter Real[n]     RollerAngles            = { (2 * alpha * (i - 1)) for i in 1 : n } "Angles between downward vertical { 0, -1, 0 } and roller center radius vectors";
  parameter Real[n, 3]  RollerCenterDirections  = { { sin(RollerAngles[i]), -cos(RollerAngles[i]), 0 } for i in 1 : n };
  parameter Real[n, 3]  RollerAxisDirections    = { { cos(RollerAngles[i]), sin(RollerAngles[i]),  0 } for i in 1 : n };
  parameter Real[n, 3]  RollerCenters           = R1 * RollerCenterDirections;
  parameter Real[n, 3]  VerticalInRollersAxes   = { { sin(RollerAngles[i]), cos(RollerAngles[i]),  0 } for i in 1 : n };

  RollerPointContactForcesGeneral[n] Contacts;

  NPortsHeavyBody[n] Rollers
      ( each N = 2
      , each Gravity = Gravity
      , r(start = { r0 + T0 * RollerCenters[i] for i in 1 : n })
      , v(start = { v0 + T0 * cross(omega0, RollerCenters[i]) for i in 1 : n })
      , q(start = { QMult(q0, { cos(RollerAngles[i] / 2), 0, 0, sin(RollerAngles[i] / 2) }) for i in 1 : n })
      , omega
          ( start
            = { { 0, 0, omega0[3] } for i in 1 : n }
            + omega0[2] * VerticalInRollersAxes
          )
      );

  FixedJoint[n] Joints
    ( each nA = { 1, 0, 0 }
    , nB = RollerAxisDirections
    , each rA = { 0, 0, 0 }
    , rB = RollerCenters
    );

  NPortsHeavyBody Wheel
    ( N = 1 + n
    , Gravity = Gravity
    , r(start = r0)
    , v(start = v0)
    , q(start = q0)
    , omega(start = omega0)
    );

  KinematicPort  InPortK;
  WrenchPort     InPortF;
  KinematicPort  OutPortK;

  Real[3]        w; // fixme: why do we need this variable ?

equation

  for i in 1 : n loop
    Contacts[i].n1k = Wheel.T * { 0, 0, 1 };
    Contacts[i].rho = (Wheel.r - Rollers[i].r) / sqrt((Wheel.r - Rollers[i].r) * (Wheel.r - Rollers[i].r));

    connect(InPortK,                 Contacts[i].InPortA);
    connect(Rollers[i].OutPort,      Contacts[i].InPortB);
    connect(Rollers[i].InPorts[1],   Contacts[i].OutPortB);

    connect(Rollers[i].InPorts[2],   Joints[i].OutPortA);
    connect(Rollers[i].OutPort,      Joints[i].InPortA);

    connect(Wheel.InPorts[i + 1],    Joints[i].OutPortB); // fixme: should it be assymmetric as it is now ?

    connect(Wheel.OutPort,           Joints[i].InPortB);
  end for;

  connect(Wheel.InPorts[1],  InPortF);
  connect(Wheel.OutPort,     OutPortK);

  w = transpose(Wheel.OutPort.T) * (Rollers[1].r - Wheel.r);

  /*
  // fixme: why is it not symmetric ???
  connect(Wheel.InPort,  Joint0.OutPortB);
  connect(Wheel.InPort1, Joint1.OutPortB);
  connect(Wheel.InPort2, Joint2.OutPortB);
  connect(Wheel.InPort3, Joint3.OutPortB);
  connect(Wheel.InPort4, InPortF);
  */

end OmniWheelGeneral;
