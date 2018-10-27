within MbsLite.Test.Omni;

package TestInitials

  function atRest
    input Params params;
    output Initials initials;
  algorithm
    initials := CreateInitials
      ( name      = "atRest"
      , omega     = 0 / 3.610781043976
      , vAbs      = 0 * 0.15
      , vDirAngle = 0
      , params    = params
      , noSlip    = true
      );
  end atRest;

  function selfRot
    input Params params;
    output Initials initials;
  algorithm
    initials := CreateInitials
      ( name      = "selfRot"
      , omega     = 1 / 3.610781043976
      , vAbs      = 0 * 0.15 // 1e-3
      , vDirAngle = 1e-3
      , params    = params
      , noSlip    = true
      );
  end selfRot;

  function straight
    input Params params;
    output Initials initials;
  algorithm
    initials := CreateInitials
      ( name      = "straight"
      , omega     = 0 / 3.610781043976
      , vAbs      = 1 * 0.15
      , vDirAngle = 5e-3
      , params    = params
      , noSlip    = true
      );
  end straight;
  
  function wrench
    input Params params;
    output Initials initials;
  algorithm
    initials := CreateInitials
      ( name      = "wrench"
      , omega     = 1 / 3.610781043976
      , vAbs      = 1 * 0.15
      , vDirAngle = 5e-3
      , params    = params
      , noSlip    = true
      );
  end wrench;

  constant Initials wheelStill = Initials
    ( name = "wheel still"
    , omega = 0
    , vAbs = 0
    , vDirAngle = 0
    , vVec = zeros(3)
    , omegaVec = zeros(3)
    , params   = CreateParamsForOneWheelSimulation()
    , noSlip   = true
    );

  constant Initials wheelSpinning = Initials
    ( name = "wheel still"
    , omega = inf
    , vAbs = 0
    , vDirAngle = 0
    , vVec = zeros(3)
    , omegaVec = 1 * vertical
    , params   = CreateParamsForOneWheelSimulation()
    , noSlip    = true
    );

  constant Initials wheelSelfRotatingInPlace = Initials
    ( name = "wheel rotating CW looking from user"
    , omega = -1
    , vAbs = 0
    , vDirAngle = 0
    , vVec = zeros(3)
    , omegaVec = -1 * userward
    , params   = CreateParamsForOneWheelSimulation()
    , noSlip    = true
    );

  constant Initials wheelSliding = Initials
    ( name = "wheel sliding forward"
    , omega = 0
    , vAbs = 1
    , vDirAngle = 0
    , vVec = 1 * forward
    , omegaVec = zeros(3)
    , params   = CreateParamsForOneWheelSimulation()
    , noSlip    = true
    );

  constant Initials wheelSlidingAlongItsAxis = Initials
    ( name = "wheel sliding forward"
    , omega = 0
    , vAbs = 1
    , vDirAngle = pi / 2
    , vVec = 1 * QToT(QRot(pi / 2, vertical)) * forward
    , omegaVec = zeros(3)
    , params   = CreateParamsForOneWheelSimulation()
    , noSlip    = true
    );

  constant Initials wheelSelfRotatingAndSliding = Initials
    ( name = "wheel rotating CW looking from user and sliding forward"
    , omega = -1
    , vAbs = 1
    , vDirAngle = 0
    , vVec = 1 * forward
    , omegaVec = -1 * userward
    , params   = CreateParamsForOneWheelSimulation()
    , noSlip    = true
    );

  constant Initials wheelRolling_R_0_05 = Initials
    ( name = "wheel rotating CW looking from user and sliding forward without slip"
    , omega = -1
    , vAbs = 0.05
    , vDirAngle = 0
    , vVec = 0.05 * forward
    , omegaVec = -1 * userward
    , params   = CreateParamsForOneWheelSimulation()
    , noSlip    = true
    );

end TestInitials;
