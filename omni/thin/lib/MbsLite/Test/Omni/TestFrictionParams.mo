within MbsLite.Test.Omni;

package TestFrictionParams
  "dry and viscous friction params are to be kept consistent for simulations to be comparable at least in the dry friction regularized zone"

  constant Real defaultFrictionGapAtEndOfRoller      = 2e-3;
  constant Real defaultViscousFrictionVelocityBound  = 1e-6;

  constant FrictionParams disabled  = CreateFrictionParams
    ( name            =  "FrictionParams rubber-concrete"

    , frictionType                  = FrictionType.none
    , dryFrictionCoeff              = -1
    , viscousFrictionVelocityBound  = -1
    , frictionGapAtEndOfRoller      = defaultFrictionGapAtEndOfRoller
    );

  constant FrictionParams impactLikeViscous = CreateFrictionParams
    ( name            =  "FrictionParams impactLikeViscous"

    , frictionType                  = FrictionType.viscous
    , dryFrictionCoeff              = 1e-1
    , viscousFrictionVelocityBound  = defaultViscousFrictionVelocityBound
    , frictionGapAtEndOfRoller      = defaultFrictionGapAtEndOfRoller
    );

  constant FrictionParams viscousRubberConcrete = CreateFrictionParams
    ( name            =  "FrictionParams viscous rubber-concrete"

    , frictionType                  = FrictionType.viscous
    , dryFrictionCoeff              = 0.85
    , viscousFrictionVelocityBound  = defaultViscousFrictionVelocityBound
    , frictionGapAtEndOfRoller      = defaultFrictionGapAtEndOfRoller
    );

  constant FrictionParams dryRubberConcrete = CreateFrictionParams
    ( name            =  "FrictionParams dry rubber-concrete"

    , frictionType                  = FrictionType.dry
    , dryFrictionCoeff              = 0.85
    , viscousFrictionVelocityBound  = defaultViscousFrictionVelocityBound
    , frictionGapAtEndOfRoller      = defaultFrictionGapAtEndOfRoller
    );

end TestFrictionParams;
