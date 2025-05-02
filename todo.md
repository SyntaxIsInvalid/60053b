# TODO List

## Implementation Tasks
- [ ] Make program style consistent 
- [ ] Add topp-ra
- [ ] Add pf, mixture model + gating + colored noise with 1st order mc for temporal correlation + band limit, ess, kld, stratified sampling
- [ ] Differential evolution for PID tuning
- [ ] Figure out motor feed forward
- [ ] Make an abstract arm class
- [ ] Figure out ray casting for MCL
- [ ] Figure out QP with HPIPM make MPC
- [ ] Make Ramsete
- [ ] Finish chassis class 
- [ ] Figure out motor dynamics 
- [ ] Add exponetial trapezoidal motion profile and 2d mp
- [ ] Figure out how to model wheel slip
- [ ] maybe add qp between local discontinuities in acceleration
- [ ] learn slew rate stuff
- [ ] figure out how to do all the path planning cause itse unoptimal paths with splines currently

## Empirical Tasks (when robot is available)
- [ ] k_s * sgn(v) + k_v * v + k_a * a regress to fit the constants 
- [ ] Add dynamic resistance assume copper winding 
- [ ] velocitymax = (ωfree * π * dwheels)/rgearing, wfree is just free spinning
- [ ] accelerationmax = (2 * n * τstall * rgearing)/(dwheels * mrobot), n = 3, rgearing is motor gearing, mrobot is mass
- [ ] kv = Vmax/velocitymax
- [ ] ka = Vmax/accelerationmax
