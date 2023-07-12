agent_pos = [35, -20];

dest_pos = [45, 190];

obs_pos = [60, 45;
         230, 40];

checkpoints = aStar(agent_pos, dest_pos, obs_pos);

% Move the agent using the checkpoints
for i = 1:size(checkpoints, 1)
    agent_pos = checkpoints(i, :);
    disp(agent_pos); % Replace this line with your actual code to move the agent in your specific environment
end