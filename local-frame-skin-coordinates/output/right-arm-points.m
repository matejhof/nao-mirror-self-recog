right_arm = [
0.0293003185829 -0.0289829757041 0.0222767563425 -0.0697564737351 -0.790928419756 0.607919623959;
0.0282347064357 4.50768311566e-14 0.0364804677524 -0.0697564737495 -2.54727890864e-12 0.997564050259;
0.0293003185829 0.0289829757042 0.0222767563425 -0.0697564737473 0.790928419754 0.60791962396;

0.038164114903 0.0161283819803 0.033493895299 -0.0697564737471 0.432795716776 0.898789130948;
0.0381641149031 -0.0161283819802 0.033493895299 -0.069756473745 -0.432795716784 0.898789130945;
0.0402608705942 -0.0363975068811 0.00825286703986 -0.0697564737512 -0.972868805277 0.220590847701;
0.0402608705942 0.0363975068812 0.00825286703986 -0.0697564737531 0.972868805277 0.220590847701;

0.0594953467838 -0.0372657006729 0.0103130960535 -0.069756473739 -0.96142644724 0.266069954936;
0.0574717991668 -0.0161660269784 0.0349689773178 -0.0697564737419 -0.418602958592 0.905486387213;
0.0574717991669 0.0161660269785 0.0349689773178 -0.0697564737495 0.418602958594 0.905486387212;
0.0594953467838 0.037265700673 0.0103130960535 -0.0697564737521 0.961426447239 0.266069954935;

0.0678816525088 -0.0295542764539 0.0258327535611 -0.0697564737414 -0.751086405149 0.656508374944;
0.0668894149521 4.69597694064e-14 0.0391834682856 -0.0697564737495 -2.54461066956e-12 0.997564050259;
0.0678816525088 0.029554276454 0.025832753561 -0.0697564737399 0.751086405151 0.656508374942;

0.0871677481579 -0.0297714032852 0.0276069322419 -0.0697564737376 -0.731473108926 0.678292802033;
0.0862109371119 3.90372179027e-14 0.0405345607321 -0.0697564737498 -2.41920343278e-12 0.997564050259;
0.0871677481579 0.0297714032853 0.0276069322419 -0.0697564737447 0.731473108917 0.678292802042;

0.0960363877578 -0.0162308700986 0.0378917018983 -0.0697564737424 -0.392787359373 0.916979893284;
0.0960363877579 0.0162308700987 0.0378917018983 -0.0697564737511 0.392787359367 0.916979893286;

%%%

0.0203671975097 0.00956579623108 -0.0346335572732 -0.0697564737469 0.265583356361 -0.961560978407;
0.0203671975097 -0.00956579623104 -0.0346335572732 -0.0697564737384 -0.265583356359 -0.961560978408;

0.0384691416614 0.0321253033488 -0.0187487842462 -0.0697564737458 0.861569427546 -0.502824179897;
0.0374149833171 0.0185066458799 -0.0321803942189 -0.069756473753 0.497315826509 -0.864760662307;
0.037414983317 -0.0185066458799 -0.0321803942189 -0.0697564737409 -0.49731582651 -0.864760662307;
0.0384691416615 -0.0321253033487 -0.0187487842462 -0.0697564737414 -0.86156942755 -0.50282417989;

0.053825858105 0.00956273064079 -0.0370559738775 -0.069756473753 0.24926685238 -0.965919287867;
0.0538258581049 -0.00956273064074 -0.0370559738775 -0.0697564737445 -0.249266852384 -0.965919287867
];

hold off;
scatter3(right_arm(:,1),right_arm(:,2),right_arm(:,3));
hold on;
for i = 1:length(right_arm)
  plot3([right_arm(i, 1);right_arm(i, 1)+right_arm(i, 4)*.01], [right_arm(i, 2);right_arm(i, 2)+right_arm(i, 5)*.01],[right_arm(i, 3);right_arm(i, 3)+right_arm(i, 6)*.01])
end
