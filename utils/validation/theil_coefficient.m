function U = theil_coefficient(y_sim, y_exp)
    MSE = mean((y_sim - y_exp).^2);
    U = sqrt(MSE) / (sqrt(mean(y_sim.^2)) + sqrt(mean(y_exp.^2)));
end
