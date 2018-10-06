within MbsLite.Examples.OmniVehicle.PointContact;

model OmniWheel

  constant  Real[3] rollerAxisLocal  = forward;
  constant  Real[3] wheelAxis        = userward;

  parameter Params   params;
  parameter Initials initials;

  parameter String       name        = "NOT INITIALIZED";
  parameter Integer      nActual     = -Integer_inf "actual number of rollers on the wheel";
  parameter Real[3]      Gravity     (each start = inf);
  parameter Real[3]      r0          (each start = inf);
  parameter Real[4]      q0          (each start = inf);
  parameter Real[3, 3]   T0          = QToT(q0);


  parameter Real[nActual]     RollerAngles            = { (2 * params.rollerHalfAngle * (i - 1)) for i in 1 : nActual } "Angles between downward vertical { 0, -1, 0 } and roller center radius vectors";
  // FIXME: OpenModelica just can't use above array :(
  // parameter Real[nActual, 4]  RollerQs                = { QMult(q0, QRot(RollerAngles[i], wheelAxis)) for i in 1 : nActual };
  parameter Real[nActual, 4]  RollerQs                = { QMult(q0, QRot(2 * params.rollerHalfAngle * (i - 1), wheelAxis)) for i in 1 : nActual };

  parameter Real[nActual, 3]  VerticalInRollersAxes   = { QToT(RollerQs[i, :]) * vertical        for i in 1 : nActual };
  parameter Real[nActual, 3]  RollerCenterDirections  = -VerticalInRollersAxes;
  parameter Real[nActual, 3]  RollerAxisDirections    = { QToT(RollerQs[i, :]) * rollerAxisLocal for i in 1 : nActual };
  parameter Real[nActual, 3]  RollerCenters           = params.wheelHubRadius * RollerCenterDirections;

  NPortsHeavyBody[nActual] Rollers
      ( name = { name + ".Rollers[" + String(i) + "]" for i in 1 : nActual }
      , each m = params.rollerMass
      , each I = diagonal({ params.rollerAxialMoi, params.rollerOrthogonalMoi, params.rollerOrthogonalMoi })
      , each N = 2
      , each Gravity = Gravity
      , r(start = { r0 + T0 * RollerCenters[i] for i in 1 : nActual })
      , v(start = { initials.vVec + T0 * cross(initials.omegaVec, RollerCenters[i]) for i in 1 : nActual })
      , q(start = { RollerQs[i,:] for i in 1 : nActual })
      , omega
          ( start
            = { { 0, 0, initials.omegaVec[3] } for i in 1 : nActual }
            + initials.omegaVec[2] * VerticalInRollersAxes
          )
      );

  FixedJoint[nActual] Joints
    ( name = { "joint" + String(i) for i in 1 : nActual }
    , each nA = { 1, 0, 0 }
    , nB = RollerAxisDirections
    , each rA = { 0, 0, 0 }
    , rB = RollerCenters
    );

  NPortsHeavyBody Wheel
    ( name = "wheel hub"
    , m = params.wheelHubMass
    , I = diagonal({ params.wheelHubOrthogonalMoi, params.wheelHubOrthogonalMoi, params.wheelHubAxialMoi })
    , N = 1 + nActual
    , Gravity = Gravity
    , r(start = r0)
    , v(start = initials.vVec)
    , q(start = q0)
    , omega(start = initials.omegaVec)
    );

  WrenchPort     InPortF   "imports forces from the platform or verticality constraint";
  KinematicPort  OutPortK  "exports kinematics to the platform or verticality constraint";

initial algorithm
  AssertInitializedS(name,  name,     "name");
  AssertInitializedI(name,  nActual,  "nActual");
  AssertInitialized (name,  r0,       "r0");
  AssertInitialized (name,  q0,       "q0");
  AssertInitialized (name,  Gravity,  "Gravity");

equation

  for i in 1 : nActual loop
    // interactions between Rollers and the Wheel hub -- via Joints
    connect(Rollers[i].InPorts[2],   Joints[i].OutPortA); // as everywhere, forces come from inside the Joints into Rollers (via OutPortA and right into second InPorts, respectively)
    connect(Rollers[i].OutPort,      Joints[i].InPortA);  // conversely, kinematics of the Rollers is being exported into Joints
    connect(Wheel.InPorts[1 + i],    Joints[i].OutPortB); // forces are also exported to the Wheel hub...
    connect(Wheel.OutPort,           Joints[i].InPortB);  // ... and kinematics is taken therefrom into the Joints
  end for;

  // interactions between the Wheel hub and outside world (most likely, the platform of the vehicle or a verticality constraint)
  connect(Wheel.InPorts[1],  InPortF);  // forces come in
  connect(Wheel.OutPort,     OutPortK); // kinematics are exported

end OmniWheel;
