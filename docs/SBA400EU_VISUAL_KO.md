# SBA-400EU / FR5 3D 모델

메인 지도는 SEER SBA-400EU를 기본 차체로 사용합니다. 공식 공개 치수 958.2 × 631.4 × 182 mm(레이저 제외), 질량 100 kg, 스캔 높이 196.5 mm를 반영했습니다. 검정 상판, 회색 차체와 휠 홈, 청록 코너등, 파란 링 구동륜·로고 패널, 대각선 LiDAR 2개와 센서·비상정지 버튼을 구현했습니다.

외형은 사진을 참고해 만든 자체 모델이며 제조사 CAD가 아닙니다. 바퀴 크기와 센서 x/y/yaw·장착부 좌표는 추정입니다. config/model.json의 verified=false로 명시했습니다. 실제 MERGE 보정값이 있으면 3D LiDAR의 x/y/yaw에도 해당 값을 사용합니다. 배포 URDF는 기본 추정값을 유지합니다.

FR5는 공식 ROS2 메시와 관절 구조를 유지하면서 사진에 맞춰 흰색 몸체와 산호색 링을 표시합니다. Windows OpenGL GPU 렌더러가 원본 메시, 매끄러운 법선과 안티앨리어싱을 사용합니다. Pillow를 설치할 수 없거나 GPU 초기화 실패 시 기본 렌더링으로 돌아갑니다.

로봇팔 개발의 표시 선택: SIM 개발 / 실기 자세 / 실기 + SIM 비교. 실기는 연결된 FR5의 최근 2초 내 6개 관절값만 사용합니다. 비교에서는 실기 원래 색과 SIM 파란 반투명을 겹쳐 봅니다. 지도 3D의 SIM 비교는 REAL 모드에서 독립된 SIM AMR·팔 자세를 파란색으로 표시합니다. 개발 제어는 SIM을 조작하며 실기 API 실행은 기존 별도 기능을 사용합니다.

ROS2 패키지: models/seer_sba400eu_description. Windows에서 URDF를 직접 가져올 수 있으며 ROS 설치가 필요하지 않습니다. ROS2 사용법은 패키지 README를 참고하세요.

공식 자료: https://seer-robotics.ai/amr/liftingrobot/SBA-400EU 및 https://www.fairino.com/FR/4.html
