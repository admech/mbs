within MbsLite.Test.Omni;

package TestFrictionParams
  "dry and viscous friction params are to be kept consistent for simulations to be comparable at least in the dry friction regularized zone"

  constant Real defaultFrictionGapAtEndOfRoller      = 2e-3;
  constant Real defaultViscousFrictionVelocityBound  = 1e-6;

  constant Real defaultWheelJointsFrictionCoeff      = 0;
  constant Real defaultRollerJointsFrictionCoeff     = 0;

  constant FrictionParams disabled  = CreateFrictionParams
    ( name            =  "FrictionParams disabled"

    , frictionType                  = FrictionType.none
    , frictionCoeff                 = inf
    , viscousFrictionVelocityBound  = 0
    , frictionGapAtEndOfRoller      = defaultFrictionGapAtEndOfRoller

    , wheelJointsFrictionCoeff      = defaultWheelJointsFrictionCoeff
    , rollerJointsFrictionCoeff     = defaultRollerJointsFrictionCoeff
    );

  constant FrictionParams impactLikeViscous = CreateFrictionParams
    ( name            =  "FrictionParams impactLikeViscous"

    , frictionType                  = FrictionType.viscous
    , frictionCoeff                 = 1e-1 / defaultViscousFrictionVelocityBound // for consistency with experiments that were run earlier
    , viscousFrictionVelocityBound  = 0
    , frictionGapAtEndOfRoller      = defaultFrictionGapAtEndOfRoller

    , wheelJointsFrictionCoeff      = defaultWheelJointsFrictionCoeff
    , rollerJointsFrictionCoeff     = defaultRollerJointsFrictionCoeff
    );

  constant FrictionParams dryRubberConcrete = CreateFrictionParams
    ( name            =  "FrictionParams dryRubberConcrete"

    , frictionType                  = FrictionType.dry
    , frictionCoeff              = 0.85
    , viscousFrictionVelocityBound  = defaultViscousFrictionVelocityBound
    , frictionGapAtEndOfRoller      = defaultFrictionGapAtEndOfRoller

    , wheelJointsFrictionCoeff      = defaultWheelJointsFrictionCoeff
    , rollerJointsFrictionCoeff     = defaultRollerJointsFrictionCoeff
    );

  constant FrictionParams impactLikeViscousJointsFrictionSmall = CreateFrictionParams
    ( name            =  "FrictionParams impactLikeViscousJointsFrictionSmall"

    , frictionType                  = FrictionType.viscous
    , frictionCoeff                 = 1e-1 / defaultViscousFrictionVelocityBound // for consistency with experiments that were run earlier
    , viscousFrictionVelocityBound  = 0
    , frictionGapAtEndOfRoller      = defaultFrictionGapAtEndOfRoller

    , wheelJointsFrictionCoeff      = defaultWheelJointsFrictionCoeff
    , rollerJointsFrictionCoeff     = 1e-5
    );

  constant FrictionParams impactLikeViscousJointsFrictionLarge = CreateFrictionParams
    ( name            =  "FrictionParams impactLikeViscousJointsFrictionLarge"

    , frictionType                  = FrictionType.viscous
    , frictionCoeff                 = 1e-1 / defaultViscousFrictionVelocityBound // for consistency with experiments that were run earlier
    , viscousFrictionVelocityBound  = 0
    , frictionGapAtEndOfRoller      = defaultFrictionGapAtEndOfRoller

    , wheelJointsFrictionCoeff      = defaultWheelJointsFrictionCoeff
    , rollerJointsFrictionCoeff     = 1e-4
    );

end TestFrictionParams;
