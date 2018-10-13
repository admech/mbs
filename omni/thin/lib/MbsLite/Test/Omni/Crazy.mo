within MbsLite.Test.Omni;

model Crazy
  import MbsLite.Examples.OmniVehicle.CalculateOmniVehicleParams;
  import MbsLite.Examples.OmniVehicle.Params;
  import MbsLite.Examples.OmniVehicle.OmniVehicleParams;

  constant Params params
    = TestParams.pmm;

  constant OmniVehicleParams ovp
    = CalculateOmniVehicleParams
        ( params             
            = params
        , initials
            = TestInitials.atRest

        , gravity             = -vertical
        , platformQuaternion  = QRot(0, vertical) 
        );

  Insane insane(
    ovp = ovp
  );

end Crazy;

