function q_init = TRIAD_Initial_Alignment(acc, mag)
    % TRIAD_Initial_Alignment: 가속도와 지자기 데이터를 이용한 초기 자세(쿼터니언) 계산
    % 
    % [Input]
    % acc : 가속도계 측정값 [ax; ay; az] (로켓 코 방향이 +9.81인 상태)
    % mag : 지자기 센서 측정값 [mx; my; mz]
    %
    % [Output]
    % q_init : 초기 쿼터니언 [qw; qx; qy; qz]

    %% 1. 기준 벡터 정의 (NED 좌표계 기준)
    S  = SensorSpec;
    r1 = [0; 0; 1];          % Reference 1: Down (중력 방향)
    r2 = S.m_ref_ned;        % Reference 2: 지자기 기준벡터 (편각 포함)

    %% 2. 관측 벡터 정의 및 부호 교정 (Body 좌표계 기준)
    % 사용자님의 센서 특성: 코(Body X)가 하늘을 향할 때 +9.81이 측정됨.
    % NED 기준인 r1(Down)과 맞추기 위해 가속도 벡터에 마이너스를 붙여 b1을 생성.
    b1 = -acc / norm(acc); % 측정된 'Up'을 'Down'으로 변환
    b2 = mag / norm(mag);  % 지자기 벡터 정규화

    %% 3. TRIAD 중간 좌표계(Triad Frame) 생성
    % NED 좌표계에서의 Triad Frame (M_ref = C_t_n)
    u1 = r1;
    u2 = cross(u1, r2) / norm(cross(u1, r2)); % East 방향 축
    u3 = cross(u1, u2);                       % South 방향 축
    M_ref = [u1, u2, u3];

    % Body 좌표계에서의 Triad Frame (M_body = C_t_b)
    v1 = b1;
    v2 = cross(v1, b2) / norm(cross(v1, b2));
    v3 = cross(v1, v2);
    M_body = [v1, v2, v3];

    %% 4. 최종 변환 행렬(DCM) 및 쿼터니언 도출
    % 연쇄 법칙: C_b_n = C_t_n * (C_t_b)'
    C_bn = M_ref * M_body';

    % DCM을 쿼터니언으로 변환 (Aerospace Toolbox 함수 사용)
    % 만약 Toolbox가 없다면 직접 수식으로 변환하는 로직이 필요합니다.
    q_init = NavUtils.dcm2quat(C_bn)'; 
    
    % 쿼터니언 정규화 (수치적 안정성 확보)
    q_init = q_init / norm(q_init);
end
