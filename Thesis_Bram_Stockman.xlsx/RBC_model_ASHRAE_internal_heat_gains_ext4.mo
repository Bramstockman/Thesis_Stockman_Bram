within GIT_thesis_Bram_Stockman;
model RBC_model_ASHRAE_internal_heat_gains_ext4
  extends RBC_model_ASHRAE_internal_heat_gains(
    HeatingCurveIn(table=[-8.0 + 273,35 + 273; 10 + 273,26.3 + 273; 15 + 273,
          23 + 273]),
    CoolingCurveIn(table=[15 + 273,19 + 273; 22 + 273,14 + 273]),
    fanEmbeddedPipe(m_flow_nominal=1.4),
    heatingCoolingSet1(HeatSet=16 + 273, CoolSet=20 + 273));
end RBC_model_ASHRAE_internal_heat_gains_ext4;
