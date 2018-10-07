within MbsLite.Test.Omni;

package TestParams
  import MbsLite.Examples.OmniVehicle.Params;

  constant Params pmm = CreateParams
    ( name            =  "params used in the PMM paper"

    , NWheels         =  3
    , nRollers        =  5
    , mecanumAngle    =  0

    , platformRadius  =  0.15
    , wheelRadius     =  0.05

    , platformMass    =  1
    , wheelHubMass    =  0.15
    , rollerMass      =  0.05
    );

end TestParams;