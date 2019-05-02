within GIT_thesis_Bram_Stockman;
model RBC_model_ASHRAE_internal_heat_gains_ext1
  extends RBC_model_ASHRAE_internal_heat_gains(
    HeatingCurveIn(table=[-8.0 + 273,35 + 273; 15 + 273,22 + 273]),
    CoolingCurveIn(table=[20 + 273,19 + 273; 23 + 273,14 + 273]),
    fanEmbeddedPipe(m_flow_nominal=1.6),
    heatingCoolingSet1(                                    CoolSet=19 + 273, HeatSet=
          19 + 273),
    CoolingCurveOut(table=[15 + 273,0; 20 + 273,-5; 25 + 273,-7]),
    HeatingCurveOut(table=[-7 + 273,4.85; 5 + 273,3; 12 + 273,0]),
    occSch(occupancy=3600*{7,17,31,41,55,65,79,89,103,113,127,137,151,161}),
    const1(k=0.0001));
end RBC_model_ASHRAE_internal_heat_gains_ext1;
