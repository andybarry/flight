function ComparePlot(num1, num2)


  for i = 1 : length(num1.x)
    clf
    
    plot(num1.x(i,:), num1.y(i,:), 'r*');
    
    hold on
    
    plot(num2.x(i,:), num2.y(i,:), 'b.');
    
    
    title(['Frame ' num2str(num1.frame_number(i))]);
    
    axis([-5 5 -5 5]);
    
    if any(num1.x(i,:)) && any(num2.x(i,:))
    
      pause
    end
    
  end



end