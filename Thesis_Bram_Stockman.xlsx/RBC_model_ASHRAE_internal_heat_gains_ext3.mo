within GIT_thesis_Bram_Stockman;
model RBC_model_ASHRAE_internal_heat_gains_ext3
  extends RBC_model_ASHRAE_internal_heat_gains(
    HeatingCurveIn(table=[-8.0 + 273,33 + 273; 15 + 273,22 + 273]),
    CoolingCurveIn(table=[20 + 273,20 + 273; 23 + 273,15 + 273]),
    fanEmbeddedPipe(m_flow_nominal=1.4));
end RBC_model_ASHRAE_internal_heat_gains_ext3;
