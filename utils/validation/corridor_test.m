function in_corridor = corridor_test(sim_data, exp_mean, exp_std)
    upper = exp_mean + 1.96*exp_std;
    lower = exp_mean - 1.96*exp_std;
    in_corridor = all(sim_data >= lower & sim_data <= upper);
end
