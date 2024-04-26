function calculateFutureTime
    % 현재 날짜와 시간 입력 받기
    currentDateTimeStr = input('현재 날짜와 시간을 입력하세요 (예: 2024-04-19 15): ', 's');
    
    % 입력된 문자열을 날짜와 시간 형식으로 변환
    currentDateTime = datetime(currentDateTimeStr, 'InputFormat', 'yyyy-MM-dd HH');
    
    % 추가할 시간(시간 단위) 입력 받기
    hoursToAdd = input('추가하고 싶은 시간(시간 단위)을 입력하세요: ');
    
end

