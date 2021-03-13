function out = ObstacleFree(q_new, q_near, obstacle)
for i = 1:length(obstacle)
    if noCollision(q_new, q_near, obstacle(i).pos)
        a(i) = 1;
    else
        a(i) = 0;
    end
end

if a
    out = 1;
else
    out = 0;
end
end