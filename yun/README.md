# yun · 로봇공학 시각 노트

Craig 교재 기반 강의 자료(Ch1–Ch3)를 따라가며 만든 인터랙티브 3D 학습 노트입니다.
수식 유도 단계마다 3D 로봇이 조립되거나 움직이고, 과제 유형 연습 문제와 모의고사를 단계별로 직접 풀어 채점할 수 있습니다.

## 열기

`visual-note/index.html`을 브라우저에서 엽니다.
3D(three.js)와 수식(MathJax)을 CDN에서 불러오므로 인터넷 연결이 필요합니다.

## 구성

- Ch1 좌표 변환: 위치·방향 표현, 동차변환, 역변환, 고정각, 축-각도
- Ch2 순기구학: D-H 파라미터, 링크 변환, D-H 빌더
- Ch3 역기구학: 대수적·기하학적 풀이, Pieper 해법(단계별 시점 이동·각도 표시)
- 계산기: CASIO fx-570ES PLUS 키 순서 안내
- 모의고사: 과제 형식의 영어 문제 8개, 풀 때마다 숫자가 새로 뽑힘

## 파일

- `visual-note/index.html`: 페이지 레이아웃과 스타일
- `visual-note/engine.js`: 3D 장면·애니메이션 엔진
- `visual-note/ch1.js`, `ch2.js`, `ch3.js`: 강의별 내용
- `visual-note/calc.js`: 계산기 사용법
- `visual-note/exam.js`: 모의고사
- `visual-note/solve.js`: 단계별 직접 풀기 채점
- `visual-note/app.js`: 화면 구성과 조작
