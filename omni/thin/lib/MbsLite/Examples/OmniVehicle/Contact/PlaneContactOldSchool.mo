within MbsLite.Examples.OmniVehicle.Contact;

model PlaneContactOldSchool
 "A -- plane, B -- roller"
  extends Constraint;

  parameter String name                        = "NOT INITIALIZED";
  parameter Params params;
  parameter Real frictionCoeff                 = inf;
  parameter Real viscousFrictionVelocityBound  = inf;

protected
  Real[3] contactPointCoords;
  Real[3] contactPointVelocity;
  Real    contactPointVelocityNorm;
  Real    cosBetweenRollerVerticalAndGlobalVertical;
  Boolean isInContact;

  Real    normalVelocity;
  Real    DnormalVelocity;
  Real    normalReaction;

  Real[3] tangentialVelocity;
  Real[3] friction;

initial equation
  AssertInitializedS(name, name,                             "name");
  AssertInitialized (name, { frictionCoeff },                "frictionCoeff");
  AssertInitialized (name, { viscousFrictionVelocityBound }, "viscousFrictionVelocityBound");

equation

  cosBetweenRollerVerticalAndGlobalVertical
    = (InPortB.T * vertical) * vertical;

  isInContact
    = cosBetweenRollerVerticalAndGlobalVertical > cos(params.rollerHalfAngle)
      and InPortB.r[2] < params.wheelRadius;
  
  // CONTACT POINT COORDS
  contactPointCoords = if isInContact
      then InPortB.r                                        // start from roller center, 
           + params.wheelHubRadius * (InPortB.T * vertical) // go to wheel center
           - params.wheelRadius * vertical                  // and then outright downward.
      else zeros(3);
  when isInContact <> pre(isInContact) then
    reinit
      ( contactPointCoords
      , if isInContact
        then InPortB.r                                        // start from roller center,
             + params.wheelHubRadius * (InPortB.T * vertical) // go to wheel center
             - params.wheelRadius * vertical                  // and then outright downward.  
        else zeros(3)
      );
  end when;

  // CONTACT POINT VELOCITY
  contactPointVelocity = if isInContact
    then Euler
           ( InPortB.r
           , contactPointCoords
           , InPortB.v
           , InPortB.omega
           )
    else zeros(3);
  contactPointVelocityNorm = norm(contactPointVelocity);

  // SIGNORINI'S LAW
  normalVelocity      = contactPointVelocity[2];
  der(normalVelocity) = DnormalVelocity;
  tangentialVelocity  = { contactPointVelocity[1], 0, contactPointVelocity[3] };
  if isInContact then
    DnormalVelocity = 0;
/*
    Assert
      ( CompareReal(0, contactPointVelocity[2], absTol = 1e-5)
      , name + " contact point has vertical speed!"
      );
*/
    friction = -frictionCoeff * contactPointVelocity
           * ( if noEvent(contactPointVelocityNorm <= viscousFrictionVelocityBound)
             then 1 / viscousFrictionVelocityBound
             else 1 / contactPointVelocityNorm
             )
           * normalReaction
           // + mu * vertical // regularization ?
           ;
  else
    normalReaction = 0;
    friction = zeros(3);
  end if;

  OutPortA.P = { contactPointCoords[1], contactPointCoords[2], contactPointCoords[3] };

  OutPortB.P = contactPointCoords;
  OutPortB.F = friction + normalReaction * vertical;
  OutPortB.M = zeros(3);

end PlaneContactOldSchool;

