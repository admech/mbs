within MbsLite.Examples.OmniVehicle;

function CalculateOmniVehicleParams

  input Params   params;
  input Initials initials;
  input Real[3]  gravity            = -vertical;
  input Real[4]  platformQuaternion = QRot(0, vertical);

  output OmniVehicleParams omniVehicleParams;

protected

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
    wheelQuaternionsRel[i] := QRot(wheelAxisAngles[i] + pi / 2, vertical);
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

      , gravity               = gravity
      , platformQuaternion    = platformQuaternion

      , platformCenter        = platformCenter
      , wheelCenters          = wheelCenters
      , wheelAxisDirections   = wheelAxisDirections
      , wheelQuaternions      = wheelQuaternionsAbs

      , wheelInitials         = wheelInitials
    );

end CalculateOmniVehicleParams;

