within MbsLite.Examples.OmniVehicle.Full;

model OmniVehicle
  import MbsLite.Examples.OmniVehicle.PointContact.OmniWheelOnPlane;

  parameter String name = "OmniVehicle";
  
  parameter Params   params;
  parameter Initials initials;

  parameter Integer  nActual            = params.nRollers;

  parameter Real[3]  gravity            = fill(inf, 3);
  parameter Real[4]  platformQuaternion = fill(inf, 4);

  // CALCULATED PARAMS

  parameter Real[params.NWheels]    wheelAxisAngles     =
    { (2 * pi / params.NWheels * (i - 1))
    for i in 1 : params.NWheels
    }
    "Angles between rightward direction (x1, i.e. forward direction) and wheel axes";
  parameter Real[params.NWheels, 4] wheelQuaternionsRel =
    // OpenModelica just can't...
    // { QRot(wheelAxisAngles[i], vertical)
    { QRot((2 * pi / params.NWheels * (i - 1)), vertical)
    for i in 1 : params.NWheels
    }
    "Directions of x1 of each wheel relative to platform";
  parameter Real[params.NWheels, 4] wheelQuaternionsAbs =
    // OpenModelica just can't...
    // { QMult(platformQuaternion, wheelQuaternionsRel[i,:])
    { QMult(platformQuaternion, QRot((2 * pi / params.NWheels * (i - 1)), vertical))
    for i in 1 : params.NWheels
    }
    "Directions of x1 of each wheel in global coords";
  parameter Real[params.NWheels, 3] wheelAxisDirections =
    // OpenModelica just can't...
    // { QToT(wheelQuaternionsAbs[i,:]) * userward
    { QToT(QMult(platformQuaternion, QRot((2 * pi / params.NWheels * (i - 1)), vertical))) * userward
    for i in 1 : params.NWheels
    };

  parameter Real[3]  platformCenter                     = params.wheelRadius * vertical;

  parameter Real[params.NWheels, 3] wheelCenters        =
    // OpenModelica just can't...
    // { platformCenter + params.platformRadius * wheelAxisDirections[i]
    { platformCenter + params.platformRadius * ( QToT(QMult(platformQuaternion, QRot((2 * pi / params.NWheels * (i - 1)), vertical))) * userward )
    for i in 1 : params.NWheels
    };

  parameter Real[params.NWheels, 3] wheelOmegasRelLocal =
    { ( (params.platformRadius * initials.omega)
      / params.wheelRadius
      * (-userward)
      )
    for i in 1 : params.NWheels
    }
    "Initial relative angular velocities of wheels in local coords";

  // OpenModelica just can't...
  /*
  parameter Real[params.NWheels, 3] wheelV0s            =
    { Euler
        ( platformCenter
        // OpenModelica just can't...
        // , wheelCenters[i]
        , platformCenter + params.platformRadius * ( QToT(QMult(platformQuaternion, QRot((2 * pi / params.NWheels * (i - 1)), vertical))) * userward )
        , initials.vVec
        , initials.omegaVec
        )
    for i in 1 : params.NWheels
    } "Initial velocities of wheels in global coords";
  */

  parameter Real[params.NWheels, 3] wheelOmega0s        =
    { initials.omegaVec + QToT(wheelQuaternionsAbs[i,:]) * wheelOmegasRelLocal[i,:]
    for i in 1 : params.NWheels
    } "Initial angular velocities of wheels in global coords";
    
  parameter Initials[params.NWheels] wheelInitials
    = { Initials
        ( name      = "OmniWheel[" + String(i) + "] initials"
        , omega     = inf
        , vAbs      = inf
        , vDirAngle = inf
        // OpenModelica just can't...
        // , vVec      = wheelV0s[i,:]
        , vVec      = 
            Euler
              ( platformCenter
              // OpenModelica just can't...
              // , wheelCenters[i]
              , platformCenter + params.platformRadius * ( QToT(QMult(platformQuaternion, QRot((2 * pi / params.NWheels * (i - 1)), vertical))) * userward )
              , initials.vVec
              , initials.omegaVec
              )
        , omegaVec  = initials.omegaVec + QToT(wheelQuaternionsAbs[i,:]) * wheelOmegasRelLocal[i,:]
        )
    for i in 1 : params.NWheels
    }
    ;

  NPortsHeavyBody platform
    ( final name    = "Platform"
    , final N       = params.NWheels
    , final m       = params.platformMass
    , final I       = diagonal({ params.platformOrthogonalMoi, params.platformAxialMoi, params.platformOrthogonalMoi })
    , final Gravity = gravity
    , final r       (start = platformCenter)
    , final v       (start = initials.vVec)
    , final q       (start = platformQuaternion)
    , final omega   (start = initials.omegaVec)
    );
 
  FixedJoint[params.NWheels] joints
    ( final    name = { ("vehicle joint " + String(i)) for i in 1 : params.NWheels }
    , each final nA = userward
    , final      nB = wheelAxisDirections
    , each final rA = zeros(3)
    , final      rB = params.platformRadius * wheelAxisDirections
    );
  OmniWheelOnPlane[params.NWheels] wheels
    ( name          = { "OmniWheel[" + String(i) + "]" for i in 1 : params.NWheels }
    , each nActual  = params.nRollers
    , each Gravity  = gravity
    , r0            = { wheelCenters[i]                for i in 1 : params.NWheels }
    , q0            = { wheelQuaternionsAbs[i]         for i in 1 : params.NWheels }
    , each params   = params
    , initials      = { wheelInitials[i]               for i in 1 : params.NWheels }
    )
    /*
    ( each final nActual  = nActual
    , each final params   = params
    )
    */
    ;

initial algorithm
  AssertInitializedS(name, name,               "name");
  AssertInitialized (name, gravity,            "gravity");
  AssertInitialized (name, platformQuaternion, "platformQuaternion");

initial equation
  for i in 1 : params.NWheels loop
    /*
    wheelInitials[i] :=
      Initials
        ( name      = "OmniWheel[" + String(i) + "] initials"
        , omega     = inf
        , vAbs      = norm(wheelV0s[i,:])
        , vDirAngle = inf
        , vVec      = wheelV0s[i,:]
        , omegaVec  = initials.omegaVec + QToT(wheelQuaternionsAbs[i,:]) * wheelOmegasRelLocal[i,:]
        );

    wheels[i].name      = "OmniWheel[" + String(i) + "]";
    wheels[i].nActual   = params.nRollers;
    wheels[i].Gravity   = gravity;
    wheels[i].r0        = wheelCenters[i,:];
    wheels[i].q0        = wheelQuaternionsAbs[i,:];
    // wheels[i].params    = params;
    wheels[i].initials  =
      // wheelInitials[i];
      Initials
        ( name      = "OmniWheel[" + String(i) + "] initials"
        , omega     = inf
        , vAbs      = norm(wheelV0s[i,:])
        , vDirAngle = inf
        , vVec      = wheelV0s[i,:]
        , omegaVec  = wheelOmega0s[i,:]
        );
    */
  end for;

equation

  assert(noEvent( platform.OutPort.v[2] * platform.OutPort.v[2] < 1e-6 ),  "Platform has vertical speed !!!" );
  assert(noEvent( platform.omega[1] * platform.omega[1] + platform.omega[3] * platform.omega[3] < 1e-3 ),  "Platform.omega is not all [2] !!!" );

  for i in 1 : params.NWheels loop
    connect( platform.OutPort,          joints[i].InPortB  );
    connect( platform.InPorts[i],       joints[i].OutPortB );
    connect( wheels[i].wheel.OutPortK,  joints[i].InPortA  );
    connect( wheels[i].wheel.InPortF,   joints[i].OutPortA );
  end for;

end OmniVehicle;

