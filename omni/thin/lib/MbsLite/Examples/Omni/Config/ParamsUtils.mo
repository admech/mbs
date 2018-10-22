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
    input Real     rollerMass;

    output Params params;

  protected
    Real rollerHalfAngle       = pi / nRollers;
    Real fatRollerHalfAngle    = pi / (if cut then (nRollers - 1) else nRollers);
    Real wheelHubRadius        = wheelRadius * cos(fatRollerHalfAngle);
    Real rollerRadius          = (wheelRadius - wheelHubRadius) / 2;
    Real rollerLength          = wheelRadius * sin(fatRollerHalfAngle);

  algorithm
    params := Params
      ( name            = name
      
      , NWheels         = NWheels
      , nRollers        = nRollers
      , mecanumAngle    = mecanumAngle
      
      , platformRadius  = platformRadius
      , wheelRadius     = wheelRadius
      
      , platformMass    = platformMass
      , wheelHubMass    = wheelHubMass
      , rollerMass      = rollerMass
    
      , rollerHalfAngle       = rollerHalfAngle
      , wheelHubRadius        = wheelHubRadius
      , rollerRadius          = rollerRadius
      , rollerLength          = rollerLength
      
      , platformAxialMoi      = CylinderAxialMoi     ( platformMass,  platformRadius )
      , platformOrthogonalMoi = CylinderOrthogonalMoi( platformMass,  platformRadius, 0.01 )
      , wheelHubAxialMoi      = CylinderAxialMoi     ( wheelHubMass,  wheelHubRadius )
      , wheelHubOrthogonalMoi = CylinderOrthogonalMoi( wheelHubMass,  wheelHubRadius, 0.01 )
      , rollerAxialMoi        = CylinderAxialMoi     ( rollerMass,    rollerRadius   )
      , rollerOrthogonalMoi   = CylinderOrthogonalMoi( rollerMass,    rollerRadius,   rollerLength )
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
    input Real          dryFrictionCoeff;
    input Real          viscousFrictionVelocityBound;
    input Real          frictionGapAtEndOfRoller;

    output FrictionParams frictionParams;

  protected
    Real viscousFrictionCoeff;

  algorithm
    // for viscous friction to be comparable with dry friction in the regularized zone
    viscousFrictionCoeff := dryFrictionCoeff / viscousFrictionVelocityBound;

    frictionParams := FrictionParams
      ( name                          = name
      , frictionType                  = frictionType
      , dryFrictionCoeff              = dryFrictionCoeff
      , viscousFrictionVelocityBound  = viscousFrictionVelocityBound
      , viscousFrictionCoeff          = viscousFrictionCoeff
      , frictionGapAtEndOfRoller      = frictionGapAtEndOfRoller
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
