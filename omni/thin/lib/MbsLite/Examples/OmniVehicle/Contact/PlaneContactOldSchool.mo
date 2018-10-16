within MbsLite.Examples.OmniVehicle.Contact;

model PlaneContactOldSchool
 "A -- plane, B -- roller"
  extends Constraint;

  parameter String name                        = "NOT INITIALIZED";
  parameter Params params;
  parameter Real frictionCoeff                 = inf;
  parameter Real viscousFrictionVelocityBound  = inf;

  parameter Boolean isInContactInitially       = false;

  Real[3] contactPointCoords                        (each stateSelect = StateSelect.never);
  Real[3] contactPointVelocity                      (each stateSelect = StateSelect.never);
  Real    contactPointVelocityNorm                  (stateSelect = StateSelect.never);
  Real    cosBetweenWheelVerticalAndGlobalVertical  (stateSelect = StateSelect.never);
  Boolean isInContact                               (stateSelect = StateSelect.never);

  Real    normalVelocity                            (stateSelect = StateSelect.never);
  Real    DnormalVelocity                           (stateSelect = StateSelect.never);
  Real    normalReaction                            (stateSelect = StateSelect.never);

  // Real    mu(start = 0, fixed = true, stateSelect = StateSelect.prefer);

  Real[3] friction                                  (each stateSelect = StateSelect.never);

  constant Real[3] rollerAxisLocal = forward;
  Real[3] rollerAxisGlobal                          (each stateSelect = StateSelect.never);
  Real[3] userwardHorizontalGlobal                  (each stateSelect = StateSelect.never);
  Real[3] towardsWheelCenterGlobal                  (each stateSelect = StateSelect.never);

equation

  cosBetweenWheelVerticalAndGlobalVertical
    = towardsWheelCenterGlobal * vertical;

  isInContact
    = cosBetweenWheelVerticalAndGlobalVertical > cos(params.rollerHalfAngle)
      and InPortB.r[2] < params.wheelRadius;
  
  // CONTACT POINT COORDS
  rollerAxisGlobal = InPortB.T * rollerAxisLocal;
  userwardHorizontalGlobal = cross(rollerAxisGlobal, vertical);
  towardsWheelCenterGlobal = normalize(cross(userwardHorizontalGlobal, rollerAxisGlobal));
  contactPointCoords = if noEvent(isInContact)
      // start from roller center, go to wheel center and then outright downward.
      then InPortB.r                                          
           + params.wheelHubRadius * towardsWheelCenterGlobal
           - params.wheelRadius * vertical                    
      else zeros(3);
  when noEvent(isInContact <> pre(isInContact)) then
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
  contactPointVelocity = if /*noEvent*/(isInContact)
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
  contactPointVelocityNorm = length(contactPointVelocity);
  if /*noEvent*/(isInContact) then
/*
    Assert
      ( CompareReal(0, contactPointVelocity[2], absTol = 1e-5)
      , name + " contact point has vertical speed!"
      );
*/
    DnormalVelocity = 0;
    friction = -frictionCoeff * contactPointVelocity
           * ( if (contactPointVelocityNorm <= viscousFrictionVelocityBound)
           // * ( if noEvent(contactPointVelocityNorm <= viscousFrictionVelocityBound)
             then 1 / viscousFrictionVelocityBound
             else 1 / contactPointVelocityNorm
             )
           * normalReaction
           // * 0
           // + mu * vertical // regularization ?
           ;
/*
    friction = zeros(3);
*/
  else
    normalReaction = 0;
    friction = zeros(3);
  end if;
  OutPortB.F = friction + normalReaction * vertical;
  // der(mu) = 0;
  // normalReaction = OutPortB.F * vertical; // e.g. for mu
  // assert(/*noEvent*/(isInContact and CompareReal(0, normalVelocity) or not isInContact), "contact " + name + " is active, but there is normal velocity: " + String(normalVelocity));

  OutPortA.P = { contactPointCoords[1], contactPointCoords[2], contactPointCoords[3] };

  OutPortB.P = contactPointCoords;
  OutPortB.M = zeros(3);

end PlaneContactOldSchool;

