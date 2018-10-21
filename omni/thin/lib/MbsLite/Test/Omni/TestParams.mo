within MbsLite.Test.Omni;

package TestParams

  constant FrictionParams frictionParams =
    TestFrictionParams.impactLikeViscous;

  constant Params pmm = CreateParams
    ( name            =  "params used in the PMM paper"

    , cut             =  true
    , NWheels         =  3
    , nRollers        =  5
    , mecanumAngle    =  0

    , platformRadius  =  0.15
    , wheelRadius     =  0.05

    , platformMass    =  1
    , wheelHubMass    =  0.15
    , rollerMass      =  0.05
    );

  constant OmniVehicleParams pmmAtRest = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.atRest
    , frictionParams
        = frictionParams

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmSelfRot = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.selfRot
    , frictionParams
        = frictionParams

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmStraight = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.straight
    , frictionParams
        = frictionParams

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmWrench = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.wrench
    , frictionParams
        = frictionParams

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmAtRestDry = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.atRest
    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmSelfRotDry = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.selfRot
    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmStraightDry = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.straight
    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmWrenchDry = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.wrench
    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmAtRestViscous = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.atRest
    , frictionParams
        = TestFrictionParams.viscousRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmSelfRotViscous = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.selfRot
    , frictionParams
        = TestFrictionParams.viscousRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmStraightViscous = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.straight
    , frictionParams
        = TestFrictionParams.viscousRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmWrenchViscous = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.wrench
    , frictionParams
        = TestFrictionParams.viscousRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

end TestParams;
