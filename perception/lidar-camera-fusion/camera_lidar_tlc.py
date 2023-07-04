import numpy as np

# 출력 형식 설정
np.set_printoptions(precision=8, suppress=True)

 # 카메라에서 라이더로 가는 방법 x 오른손 법칙마냥 오른손의 축방향일때 나머지 손가락이 +임 
a = np.deg2rad(18) # 카메라 좌표계  x축
b = np.deg2rad(30) # 카메라 좌표계 y축

Rx = np.array([[1, 0, 0,0],
      [0, np.cos(a), -np.sin(a),0],
      [0, np.sin(a), np.cos(a),0],
      [0,0,0,1]])

Ryb = np.array([[np.cos(b), 0, np.sin(b),0],
      [0, 1, 0,0],
      [-np.sin(b), 0, np.cos(b),0],
      [0,0,0,1]])

Ry90 = np.array([[np.cos(np.deg2rad(-90)), 0, np.sin(np.deg2rad(-90)),0],
      [0, 1, 0,0],
      [-np.sin(np.deg2rad(-90)), 0, np.cos(np.deg2rad(-90)),0],
      [0,0,0,1]])

Rx90= np.array([[1, 0, 0,0],
         [0, np.cos(np.deg2rad(90)), -np.sin(np.deg2rad(90)),0],
         [0, np.sin(np.deg2rad(90)), np.cos(np.deg2rad(90)),0],
         [0,0,0,1]])

# 라이더로 기준으로  
T = np.array([[1, 0, 0, 0.93],
     [0, 1, 0, -0.18],
     [0, 0, 1, -0.62],
     [0, 0, 0, 1]])

print(Rx@Ryb@Ry90@Rx90@T) #카메라를 기준으로 라이더를 본 회전행렬 