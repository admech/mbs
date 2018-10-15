within MbsLite.Examples.OmniVehicle.Contact;

model PlaneContactOldSchool
 "A -- plane, B -- roller"
  extends Constraint;

  parameter String name                        = "NOT INITIALIZED";
  parameter Params params;
  parameter Real frictionCoeff                 = inf;
  parameter Real viscousFrictionVelocityBound  = inf;

  parameter Boolean isInContactInitially       = false;

// protected
  Real[3] contactPointCoords;
  Real[3] contactPointVelocity;
  Real    contactPointVelocityNorm;
  Real    cosBetweenWheelVerticalAndGlobalVertical;
  Boolean isInContact;

  Real    normalVelocity;
  Real    DnormalVelocity;
  Real    normalReaction;

  // Real    mu(start = 0, fixed = true, stateSelect = StateSelect.prefer);

  Real[3] friction;

  constant Real[3] rollerAxisLocal = forward;
  Real[3] rollerAxisGlobal;
  Real[3] userwardHorizontalGlobal;
  Real[3] towardsWheelCenterGlobal;

equation

  isInContact
    = cosBetweenWheelVerticalAndGlobalVertical > cos(params.rollerHalfAngle)
      and InPortB.r[2] < params.wheelRadius;
  
  // CONTACT POINT COORDS
  rollerAxisGlobal = InPortB.T * rollerAxisLocal;
  userwardHorizontalGlobal = cross(rollerAxisGlobal, vertical);
  towardsWheelCenterGlobal = cross(userwardHorizontalGlobal / norm(userwardHorizontalGlobal), rollerAxisGlobal);
  cosBetweenWheelVerticalAndGlobalVertical
    = towardsWheelCenterGlobal * vertical;

  contactPointCoords = if isInContact
      then InPortB.r                                          // start from roller center, 
           + params.wheelHubRadius * towardsWheelCenterGlobal // go to wheel center
           - params.wheelRadius * vertical                    // and then outright downward.
      else zeros(3);
  when isInContact <> pre(isInContact) then
    print("REINITIALIZING " + name);
    reinit
      ( contactPointCoords
      , if isInContact
        then InPortB.r                                          // start from roller center,
             + params.wheelHubRadius * towardsWheelCenterGlobal // go to wheel center
             - params.wheelRadius * vertical                    // and then outright downward.  
        else zeros(3)
      );
/*
    reinit
      ( normalVelocity
      , if isInContact then 0 else normalVelocity
      );
    reinit
      ( normalReaction
      , if isInContact then normalReaction else 0
      );
*/
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

  // SIGNORINI'S LAW
  normalVelocity      = contactPointVelocity[2];
  der(normalVelocity) = DnormalVelocity;
  contactPointVelocityNorm = norm(contactPointVelocity);
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
           // * 0
           // + mu * vertical // regularization ?
           ;
  else
    normalReaction = 0;
    friction = zeros(3);
  end if;
  OutPortB.F = friction + normalReaction * vertical;
  // normalReaction = OutPortB.F * vertical; // e.g. for mu

  OutPortA.P = { contactPointCoords[1], contactPointCoords[2], contactPointCoords[3] };

  OutPortB.P = contactPointCoords;
  OutPortB.M = zeros(3);

end PlaneContactOldSchool;

