within MbsLite.Examples.Omni.Config;

record FrictionParams
  "viscousFrictionCoeff should be == dryFrictionCoeff / viscousFrictionVelocityBound for viscous friction to be comparable with dry friction in the regularized zone"

  parameter String   name;

  parameter FrictionType frictionType          "dry, viscous or none";

  parameter Real dryFrictionCoeff;
  parameter Real viscousFrictionVelocityBound  "for dry friction only, specifies the size of regularization zone";
  parameter Real viscousFrictionCoeff;

  parameter Real frictionGapAtEndOfRoller      "a small angle such that friction can be turned off a bit in advance before the tip of the roller is reached";

  parameter Boolean rollerJointsHaveFriction;
  parameter Real    rollerJointsFrictionCoeff;
  parameter Boolean wheelJointsHaveFriction;
  parameter Real    wheelJointsFrictionCoeff;

end FrictionParams;
