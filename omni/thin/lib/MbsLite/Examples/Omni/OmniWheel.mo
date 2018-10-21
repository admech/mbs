within MbsLite.Examples.Omni;

model OmniWheel

  constant  Real[3] rollerAxisLocal  = forward;
  constant  Real[3] wheelAxis        = userward;

  parameter Params   params;
  parameter Initials initials;

  parameter String       name        = "NOT INITIALIZED";
  parameter Integer      nActual     = -Integer_inf "actual number of rollers on the wheel";
  parameter Real[3]      Gravity     = fill(inf, 3);
  parameter Real[3]      r0          = fill(inf, 3);
  parameter Real[4]      q0          = fill(inf, 4);
  parameter Real[3, 3]   T0          = QToT(q0);


  parameter Real[nActual]     RollerAngles            = { (2 * params.rollerHalfAngle * (i - 1)) for i in 1 : nActual } "Angles between downward vertical { 0, -1, 0 } and roller center radius vectors";
  // FIXME: OpenModelica just can't use above array :(
  // parameter Real[nActual, 4]  RollerQsRel             = { QRot(RollerAngles[i], wheelAxis) for i in 1 : nActual };
  parameter Real[nActual, 4]  RollerQsRel             = { QRot(2 * params.rollerHalfAngle * (i - 1), wheelAxis) for i in 1 : nActual };
  // FIXME: OpenModelica just can't use above array :(
  // parameter Real[nActual, 4]  RollerQsAbs             = { QMult(q0, RollerQsRel[i, :]) for i in 1 : nActual };
  parameter Real[nActual, 4]  RollerQsAbs             = { QMult(q0, QRot(2 * params.rollerHalfAngle * (i - 1), wheelAxis)) for i in 1 : nActual };

  parameter Real[nActual, 3]  RollerVerticalsInWheelCoords       = { QToT(RollerQsRel[i, :]) * vertical        for i in 1 : nActual };
  parameter Real[nActual, 3]  RollerCenterDirections             = -RollerVerticalsInWheelCoords;
  parameter Real[nActual, 3]  RollerAxisDirectionsInWheelCoords  = { QToT(RollerQsRel[i, :]) * rollerAxisLocal for i in 1 : nActual };
  parameter Real[nActual, 3]  RollerCentersInWheelCoords         = params.wheelHubRadius * RollerCenterDirections;

  NPortsHeavyBody[nActual] Rollers
      ( final name = { name + ".Rollers[" + String(i) + "]" for i in 1 : nActual }
      , each final m = params.rollerMass
      , each final I = diagonal({ params.rollerAxialMoi, params.rollerOrthogonalMoi, params.rollerOrthogonalMoi })
      , each final N = 2
      , each final Gravity = Gravity
      , final r(start = { r0 + T0 * RollerCentersInWheelCoords[i] for i in 1 : nActual })
      , final v(start = { initials.vVec + cross(initials.omegaVec, T0 * RollerCentersInWheelCoords[i]) for i in 1 : nActual })
      , final q(start = { RollerQsAbs[i,:] for i in 1 : nActual })
      , final omega
          ( start
            = { transpose(QToT(RollerQsAbs[i,:])) * initials.omegaVec
              for i in 1 : nActual
              }
          )
      );

  FixedJoint[nActual] Joints
    ( final name = { name + " joint" + String(i) for i in 1 : nActual }
    , each final nA = { 1, 0, 0 }
    , final nB = RollerAxisDirectionsInWheelCoords
    , each final rA = { 0, 0, 0 }
    , final rB = RollerCentersInWheelCoords
    );

  NPortsHeavyBody Wheel
    ( final name = "wheel hub"
    , final m = params.wheelHubMass
    , final I = diagonal({ params.wheelHubOrthogonalMoi, params.wheelHubOrthogonalMoi, params.wheelHubAxialMoi })
    , final N = 1 + nActual
    , final Gravity = Gravity
    , final r(start = r0)
    , final v(start = initials.vVec)
    , final q(start = q0)
    , final omega(start = transpose(QToT(q0)) * initials.omegaVec)
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
