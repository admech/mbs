within MbsLite.Examples.Omni.Config;

package ParamsUtils
  
  function CreateParams
    input String   name;
    input Boolean  cut = false;
  
    input Integer  NWheels;
    input Integer  nRollers;
    input Real     mecanumAngle;
  
    input Real     platformRadius;
    input Real     wheelRadius;
  
    input Real     platformMass;
    input Real     wheelHubMass;
    input Real     rollerMass                       = -1;
    input Real     fractionOfRollerMassInWholeWheel = -1;

    output Params params;

  protected
    Real rollerHalfAngle       = pi / nRollers;
    Real fatRollerHalfAngle    = pi / (if cut then (nRollers - 1) else nRollers);
    Real wheelHubRadius        = wheelRadius * cos(fatRollerHalfAngle);
    Real rollerRadiusForMoi    = (wheelRadius - wheelHubRadius) / 2;
    Real rollerLengthForMoi    = 2 * wheelRadius * sin(rollerHalfAngle);
    Real actualRollerMass;
    Real actualFractionOfRollerMassInWholeWheel;

  algorithm
    if rollerMass == -1 then
      assert(fractionOfRollerMassInWholeWheel > 0, "fractionOfRollerMassInWholeWheel should be positive, was: " + String(fractionOfRollerMassInWholeWheel));
      actualRollerMass := fractionOfRollerMassInWholeWheel * wheelHubMass / (1 - nRollers * fractionOfRollerMassInWholeWheel);
      actualFractionOfRollerMassInWholeWheel := fractionOfRollerMassInWholeWheel;
    elseif fractionOfRollerMassInWholeWheel == -1 then
      assert(rollerMass > 0, "rollerMass should be positive, was: " + String(rollerMass));
      actualRollerMass := rollerMass;
      actualFractionOfRollerMassInWholeWheel := wheelHubMass + nRollers * rollerMass;
    else
      assert(false, "only one of rollerMass and fractionOfRollerMassInWholeWheel should be specified");
    end if;

    params := Params
      ( name            = name
      
      , NWheels         = NWheels
      , nRollers        = nRollers
      , mecanumAngle    = mecanumAngle
      
      , platformRadius  = platformRadius
      , wheelRadius     = wheelRadius
      
      , platformMass    = platformMass
      , wheelHubMass    = wheelHubMass
      , rollerMass      = actualRollerMass
      , fractionOfRollerMassInWholeWheel = actualFractionOfRollerMassInWholeWheel
    
      , rollerHalfAngle       = rollerHalfAngle
      , wheelHubRadius        = wheelHubRadius
      , rollerRadiusForMoi    = rollerRadiusForMoi
      , rollerLengthForMoi    = rollerLengthForMoi
      
      , platformAxialMoi      = CylinderAxialMoi     ( platformMass,  platformRadius )
      , platformOrthogonalMoi = CylinderOrthogonalMoi( platformMass,  platformRadius, 0.01 )
      , wheelHubAxialMoi      = CylinderAxialMoi     ( wheelHubMass,  wheelHubRadius )
      , wheelHubOrthogonalMoi = CylinderOrthogonalMoi( wheelHubMass,  wheelHubRadius, 0.01 )
      , rollerAxialMoi        = CylinderAxialMoi     ( rollerMass,    rollerRadiusForMoi   )
      , rollerOrthogonalMoi   = CylinderOrthogonalMoi( rollerMass,    rollerRadiusForMoi,   rollerLengthForMoi )
      );

  end CreateParams;

  function CreateInitials
    input String   name;

    input Real     omega;
    input Real     vAbs;
    input Real     vDirAngle;

    output Initials initials;

  algorithm

    initials := Initials
      ( name      = name

      , omega     = omega
      , vAbs      = vAbs
      , vDirAngle = vDirAngle

      , vVec      = vAbs * QToT(QRot(vDirAngle, vertical)) * forward
      , omegaVec  = omega * vertical
      );

  end CreateInitials;

  function CreateFrictionParams
    input String        name;

    input FrictionType  frictionType;
    input Real          frictionCoeff;
    input Real          viscousFrictionVelocityBound "for dry friction only";
    input Real          frictionGapAtEndOfRoller;

    input Real          rollerJointsFrictionCoeff;
    input Real          wheelJointsFrictionCoeff;

    output FrictionParams frictionParams;

  protected
    Boolean rollerJointsHaveFriction;
    Boolean wheelJointsHaveFriction;

  algorithm
    rollerJointsHaveFriction := CompareReal(0, rollerJointsFrictionCoeff);
    wheelJointsHaveFriction  := CompareReal(0, wheelJointsFrictionCoeff);

    frictionParams := FrictionParams
      ( name                          = name
      , frictionType                  = frictionType
      , dryFrictionCoeff              = frictionCoeff
      , viscousFrictionVelocityBound  = viscousFrictionVelocityBound
      , viscousFrictionCoeff          = frictionCoeff
      , frictionGapAtEndOfRoller      = frictionGapAtEndOfRoller
      , rollerJointsHaveFriction      = rollerJointsHaveFriction
      , rollerJointsFrictionCoeff     = rollerJointsFrictionCoeff
      , wheelJointsHaveFriction       = wheelJointsHaveFriction
      , wheelJointsFrictionCoeff      = wheelJointsFrictionCoeff
      );

  end CreateFrictionParams;

  function CalculateOmniVehicleParams
  
    input Params         params;
    input Initials       initials;
    input FrictionParams frictionParams;
    input Real[3]  gravity            = -vertical;
    input Real[4]  platformQuaternion = QRot(0, vertical);
  
    output OmniVehicleParams omniVehicleParams;
  
  protected
  
    constant Real wheelNoise = 2e-3 "to reduce the probabiilty of multiple simultaneous impacts";
  
    Integer           nActual = params.nRollers;
    Integer           NActual = params.NWheels;
  
    Real[3]           platformCenter;
    Real[NActual, 3]  wheelCenters;
    Real[NActual, 3]  wheelAxisDirections;
  
    Initials[NActual] wheelInitials;
  
    Real[NActual]     wheelAxisAngles      "Angles between rightward direction (x1, i.e. forward direction) and wheel axes";
    Real[NActual, 4]  wheelQuaternionsRel  "Directions of x1 of each wheel relative to platform";
    Real[NActual, 4]  wheelQuaternionsAbs  "Directions of x1 of each wheel in global coords";
  
    Real[3]           wheelOmegaRelLocal   "Initial relative angular velocities of wheels in local coords";
    Real[NActual, 3]  wheelV0s             "Initial velocities of wheels in global coords";
    Real[NActual, 3]  wheelOmega0s         "Initial angular velocities of wheels in global coords";
   
  algorithm
    nActual            := params.nRollers;
    NActual            := params.NWheels;
    platformCenter     := params.wheelRadius * vertical;
  
    for i in 1 : NActual loop
      wheelAxisAngles[i]     := (2 * pi / NActual * (i - 1));
      wheelQuaternionsRel[i] := QMult
        ( QRot(wheelAxisAngles[i] + pi / 2, vertical)
        , QRot((i - 1) * wheelNoise, userward)
        );
      wheelQuaternionsAbs[i] := QMult(platformQuaternion, wheelQuaternionsRel[i,:]);
      wheelAxisDirections[i] := QToT(wheelQuaternionsAbs[i,:]) * userward;
    
      wheelCenters[i]        := platformCenter + params.platformRadius * wheelAxisDirections[i];
  
      wheelOmegaRelLocal     := (params.platformRadius*initials.omega) / params.wheelRadius * (-userward);
      wheelV0s[i]            :=
        Euler
          ( platformCenter
          , wheelCenters[i]
          , initials.vVec
          , initials.omegaVec
          );
      wheelOmega0s[i]        := initials.omegaVec + QToT(wheelQuaternionsAbs[i,:]) * wheelOmegaRelLocal;
  
      wheelInitials[i]       :=
        Initials
          ( name      = "OmniWheel[" + String(i) + "] initials"
          , omega     = inf
          , vAbs      = inf
          , vDirAngle = inf
          , vVec      = wheelV0s[i,:]
          , omegaVec  = wheelOmega0s[i]
          );
  
    end for;
  
    omniVehicleParams := OmniVehicleParams
        ( params                = params
        , initials              = initials
        , frictionParams        = frictionParams
  
        , gravity               = gravity
        , platformQuaternion    = platformQuaternion
  
        , platformCenter        = platformCenter
        , wheelCenters          = wheelCenters
        , wheelAxisDirections   = wheelAxisDirections
        , wheelQuaternions      = wheelQuaternionsAbs
  
        , wheelInitials         = wheelInitials
      );
  
  end CalculateOmniVehicleParams;

end ParamsUtils;
