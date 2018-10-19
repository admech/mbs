within MbsLite.Examples.OmniVehicle.Full;

model OmniVehicle
  import MbsLite.Examples.OmniVehicle.PointContact.OmniWheelOnPlane;
  import MbsLite.Examples.OmniVehicle.Utils.CreateRandomNoise;

  parameter String name = "OmniVehicle";
  
  parameter OmniVehicleParams ovp;

  parameter Integer  nActual            = ovp.params.nRollers;
  parameter Integer  NActual            = ovp.params.NWheels;

  NPortsHeavyBody platform
    ( name    = "Platform"
    , N       = NActual // + 1
    , m       = ovp.params.platformMass
    , I       = diagonal(
                  { ovp.params.platformOrthogonalMoi
                  , ovp.params.platformAxialMoi
                  , ovp.params.platformOrthogonalMoi
                  })
    , Gravity = ovp.gravity
    , r       (start = ovp.platformCenter)
    , v       (start = ovp.initials.vVec)
    , q       (start = ovp.platformQuaternion)
    , omega   (start = ovp.initials.omegaVec)
    );
 
  FixedJoint[NActual] joints
    (    name = { ("vehicle joint " + String(i))          for i in 1 : NActual }
    , each nA = userward
    , each rA = zeros(3)
    ,      nB = { ovp.wheelAxisDirections[i,:]            for i in 1 : NActual }
    ,      rB = { ovp.wheelCenters[i,:]                   for i in 1 : NActual }
    );
  OmniWheelOnPlane[NActual] wheels
    ( name          = { "OmniWheel[" + String(i) + "]"    for i in 1 : NActual }
    , each nActual  = ovp.params.nRollers
    , each Gravity  = ovp.gravity
    , each params   = ovp.params
    , r0            = { ovp.wheelCenters[i,:]             for i in 1 : NActual }
    , q0            = { ovp.wheelQuaternions[i,:]         for i in 1 : NActual }
    , initials      = { ovp.wheelInitials[i]              for i in 1 : NActual }
    , maxAngleNoise = { 1e-3 * (i - 1)                    for i in 1 : NActual }
    );

equation

/*
  assert(noEvent( platform.OutPort.v[2] * platform.OutPort.v[2] < 1e-6 ),  "Platform has vertical speed !!!" );
  assert(noEvent( platform.omega[1] * platform.omega[1] + platform.omega[3] * platform.omega[3] < 1e-3 ),  "Platform.omega is not all [2] !!!" );
*/

  for i in 1 : NActual loop
    connect( platform.OutPort,          joints[i].InPortB  );
    connect( platform.InPorts[i],       joints[i].OutPortB );
    connect( wheels[i].wheel.OutPortK,  joints[i].InPortA  );
    connect( wheels[i].wheel.InPortF,   joints[i].OutPortA );
  end for;

/*
  platform.InPorts[NActual + 1].P = platform.OutPort.r;
  platform.InPorts[NActual + 1].M = zeros(3);
  platform.OutPort.a = zeros(3); // Signorini no-fall-through
*/

end OmniVehicle;

