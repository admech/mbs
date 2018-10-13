within MbsLite.Examples.OmniVehicle.Full;

model OmniVehicle
  import MbsLite.Examples.OmniVehicle.PointContact.OmniWheelOnPlane;

  parameter String name = "OmniVehicle";
  
  parameter Params   params;
  parameter Initials initials;

  parameter Integer  nActual            = params.nRollers;
  parameter Integer  NActual            = params.NWheels;

  parameter Real[3]  gravity            = fill(inf, 3);
  parameter Real[4]  platformQuaternion = fill(inf, 4);

  // CALCULATED PARAMS

  parameter Real[NActual]    wheelAxisAngles(each fixed = false)
    "Angles between rightward direction (x1, i.e. forward direction) and wheel axes";
  parameter Real[NActual, 4] wheelQuaternionsRel(each fixed = false)
    "Directions of x1 of each wheel relative to platform";
  parameter Real[NActual, 4] wheelQuaternionsAbs (each fixed = false)
    "Directions of x1 of each wheel in global coords";
  parameter Real[NActual, 3] wheelAxisDirections(each fixed = false);

  parameter Real[3]  platformCenter(each fixed = false);

  parameter Real[NActual, 3] wheelCenters(each fixed = false);
  parameter Real[NActual, 3] wheelOmegasRelLocal(each fixed = false)
    "Initial relative angular velocities of wheels in local coords";
  parameter Real[NActual, 3] wheelV0s(each fixed = false)
    "Initial velocities of wheels in global coords";
  parameter Real[NActual, 3] wheelOmega0s(each fixed = false)
    "Initial angular velocities of wheels in global coords";
    
  // parameter Initials[NActual] wheelInitials;

  NPortsHeavyBody platform
    ( name    = "Platform"
    , N       = NActual
    , m       = params.platformMass
    , I       = diagonal({ params.platformOrthogonalMoi, params.platformAxialMoi, params.platformOrthogonalMoi })
    , Gravity = gravity
/*
    , r       (start = platformCenter)
    , v       (start = initials.vVec)
    , q       (start = platformQuaternion)
    , omega   (start = initials.omegaVec)
*/
    );
 
  FixedJoint[NActual] joints
    (    name = { ("vehicle joint " + String(i)) for i in 1 : NActual }
    , each nA = userward
    , each rA = zeros(3)
/*
    ,      nB = { wheelAxisDirections[i,:] for i in 1 : NActual }
    ,      rB = { (params.platformRadius * wheelAxisDirections[i,:]) for i in 1 : NActual }
*/
    );
  OmniWheelOnPlane[NActual] wheels
    ( name          = { "OmniWheel[" + String(i) + "]" for i in 1 : NActual }
    , each nActual  = params.nRollers
    , each Gravity  = gravity
    , each params   = params
/*
    , r0            = { wheelCenters[i,:]                for i in 1 : NActual }
    , q0            = { wheelQuaternionsAbs[i,:]         for i in 1 : NActual }
    , initials      = { wheelInitials[i]               for i in 1 : NActual }
*/
    );

initial algorithm
  AssertInitializedS(name, name,               "name");
  AssertInitialized (name, gravity,            "gravity");
  AssertInitialized (name, platformQuaternion, "platformQuaternion");

  platformCenter         := params.wheelRadius * vertical;

  for i in 1 : NActual loop
    wheelAxisAngles[i]     := (2 * pi / NActual * (i - 1));
    wheelQuaternionsRel[i] := QRot(wheelAxisAngles[i], vertical);
    wheelQuaternionsAbs[i] := QMult(platformQuaternion, wheelQuaternionsRel[i,:]);
    wheelAxisDirections[i] := QToT(wheelQuaternionsAbs[i,:]) * userward;
  
    wheelCenters[i]        := platformCenter + params.platformRadius * wheelAxisDirections[i];

    wheelOmegasRelLocal[i] := (params.platformRadius*initials.omega) / params.wheelRadius * (-userward);
    wheelV0s[i]            :=
      Euler
        ( platformCenter
        , wheelCenters[i]
        , initials.vVec
        , initials.omegaVec
        );
    wheelOmega0s[i]        :=
      initials.omegaVec + QToT(wheelQuaternionsAbs[i,:]) * wheelOmegasRelLocal[i,:];

/*
    wheelInitials[i]       :=
      Initials
        ( name      = "OmniWheel[" + String(i) + "] initials"
        , omega     = inf
        , vAbs      = inf
        , vDirAngle = inf
        , vVec      = wheelV0s[i,:]
        , omegaVec  = initials.omegaVec + QToT(wheelQuaternionsAbs[i,:]) * wheelOmegasRelLocal[i,:]
        );
*/

  end for;

initial equation

  platform.r             = platformCenter;
  platform.v             = initials.vVec;
  platform.q             = platformQuaternion;
  platform.omega         = initials.omegaVec;

  for i in 1 : NActual loop
    // wheels[i].params       = params;

    joints[i].nB           = wheelAxisDirections[i,:];
    joints[i].rB           = params.platformRadius * wheelAxisDirections[i,:];

    wheels[i].r0           = wheelCenters[i,:];
    wheels[i].q0           = wheelQuaternionsAbs[i,:];
    // wheels[i].initials     = wheelInitials[i];
/*
    wheels[i].initials.name = wheelInitials[i].name;
    wheels[i].initials.omega = wheelInitials[i].omega;
    wheels[i].initials.vAbs = wheelInitials[i].vAbs;
    wheels[i].initials.vDirAngle = wheelInitials[i].vDirAngle;
    wheels[i].initials.vVec = wheelInitials[i].vVec;
    wheels[i].initials.omegaVec = wheelInitials[i].omegaVec;
*/
    wheels[i].initials.name      = "OmniWheel[" + String(i) + "] initials";
    wheels[i].initials.omega     = inf;
    wheels[i].initials.vAbs      = inf;
    wheels[i].initials.vDirAngle = inf;
    wheels[i].initials.vVec      = wheelV0s[i,:];
    wheels[i].initials.omegaVec  = initials.omegaVec + QToT(wheelQuaternionsAbs[i,:]) * wheelOmegasRelLocal[i,:];
  end for;

equation

  assert(noEvent( platform.OutPort.v[2] * platform.OutPort.v[2] < 1e-6 ),  "Platform has vertical speed !!!" );
  assert(noEvent( platform.omega[1] * platform.omega[1] + platform.omega[3] * platform.omega[3] < 1e-3 ),  "Platform.omega is not all [2] !!!" );

  for i in 1 : NActual loop
    connect( platform.OutPort,          joints[i].InPortB  );
    connect( platform.InPorts[i],       joints[i].OutPortB );
    connect( wheels[i].wheel.OutPortK,  joints[i].InPortA  );
    connect( wheels[i].wheel.InPortF,   joints[i].OutPortA );
  end for;

end OmniVehicle;

