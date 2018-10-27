within MbsLite.Examples.Omni;

model OmniVehicle

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
    , each final hasFriction   = ovp.frictionParams.wheelJointsHaveFriction
    , each final frictionCoeff = ovp.frictionParams.wheelJointsFrictionCoeff
    );
  OmniWheelOnPlane[NActual] wheels
    ( name          = { "OmniWheel[" + String(i) + "]"    for i in 1 : NActual }
    , each nActual  = ovp.params.nRollers
    , each Gravity  = ovp.gravity
    , each params   = ovp.params
    , r0            = { ovp.wheelCenters[i,:]             for i in 1 : NActual }
    , q0            = { ovp.wheelQuaternions[i,:]         for i in 1 : NActual }
    , initials      = { ovp.wheelInitials[i]              for i in 1 : NActual }
    , each frictionParams = ovp.frictionParams
    , wheelAxialOmega0       = { ovp.initials.wheelAxialOmegas[i]        for i in 1 : NActual }
    , firstRollerAxialOmega0 = { ovp.initials.firstRollerAxialOmegas[i]  for i in 1 : NActual }
    );

  TelemetryInfo telemetry 
    ( NWheels  = ovp.params.NWheels
    , nRollers = ovp.params.nRollers
    );
  Real DTheta (stateSelect = StateSelect.never);

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
  when { pre(wheels[i].indexOfRollerInContact) <> wheels[i].indexOfRollerInContact for i in 1 : NActual } then
    reinit(platform.r[2], ovp.params.wheelRadius);
    reinit(platform.v[2], 0);
  end when;
*/

  // TELEMETRY

  telemetry.kineticEnergy                  = platform.kineticEnergy + sum
    ({ wheels[i].wheel.Wheel.kineticEnergy
       + sum
           ({ wheels[i].wheel.Rollers[j].kineticEnergy
           for j in 1 : nActual
           })
    for i in 1 : NActual
    });
  telemetry.centerVelocityNorm             = length(platform.OutPort.v);
  telemetry.platformAngularVelocity        = platform.OutPort.omega * vertical;

  telemetry.indicesOfRollerInContact       = 
    ({ wheels[i].indexOfRollerInContact
    for i in 1 : NActual
    });
  telemetry.contactPointVelocitiesSigned   = 
    ({ wheels[i].contactPointVelocity
    for i in 1 : NActual
    });
  telemetry.frictionNormsSigned            = 
    ({ wheels[i].friction
    for i in 1 : NActual
    });
  telemetry.normalReactions                     = 
    { wheels[i].normalReaction
    for i in 1 : NActual
    };

  telemetry.lagrangianCoords.x           =  platform.OutPort.r[1]; 
  telemetry.lagrangianCoords.y           = -platform.OutPort.r[3];
  der(telemetry.lagrangianCoords.theta)  = DTheta;
  der(DTheta)                            = platform.OutPort.omega[2];

  telemetry.lagrangianCoords.chi         =
    { joints[i].angle
    for i in 1 : NActual
    };
  telemetry.lagrangianCoords.phi         =
    { { wheels[i].wheel.Joints[j].angle
      for j in 1 : nActual
      }
    for i in 1 : NActual
    };
  telemetry.lagrangianCoords.Dphi        =
    { { wheels[i].wheel.Joints[j].mu
      for j in 1 : nActual
      }
    for i in 1 : NActual
    };

  telemetry.debuggingInfo.centerAltitude                      = platform.OutPort.r[2]; 
  telemetry.debuggingInfo.centerVerticalVelocity              = platform.OutPort.v[2]; 
  telemetry.debuggingInfo.platformNonVerticalAngularVelocity  = length
    ({ platform.OutPort.omega[1], 0, platform.OutPort.omega[3] });

  telemetry.debuggingInfo.wheelAltitudes                      = 
    { wheels[i].wheel.Wheel.OutPort.r[2]
    for i in 1 : NActual
    };
  telemetry.debuggingInfo.wheelVerticalVelocities             = 
    { wheels[i].wheel.Wheel.OutPort.v[2]
    for i in 1 : NActual
    };
  telemetry.debuggingInfo.contactNormalVelocities             = 
    { wheels[i].contactPointNormalVelocity
    for i in 1 : NActual
    };
  
end OmniVehicle;


