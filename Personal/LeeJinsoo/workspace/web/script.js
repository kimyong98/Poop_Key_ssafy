function createGrid(containerId, n, m) {
    const container = document.getElementById(containerId);
    container.innerHTML = ''; // 기존 내용을 지웁니다.
    for (let i = 0; i < m; i++) {
        const row = document.createElement('div');
        row.classList.add('row');
        for (let j = 0; j < n; j++) {
            const square = document.createElement('div');
            square.classList.add('square');
            square.dataset.row = i;
            square.dataset.col = j;
            row.appendChild(square);
        }
        container.appendChild(row);
    }
}

function changeColor(containerId, row, col, color) {
    const container = document.getElementById(containerId);
    const squares = container.querySelectorAll(`.square[data-row="${row}"][data-col="${col}"]`);
    if (squares.length > 0) {
        squares[0].style.backgroundColor = color;
    } else {
        console.error('Invalid coordinates or container ID');
    }
}

function hideBox(containerId, row, col) {
    const container = document.getElementById(containerId);
    const square = container.querySelector(`.square[data-row="${row}"][data-col="${col}"]`);
    if (square) {
        square.classList.add('hidden');
    } else {
        console.error('Invalid coordinates or container ID');
    }
}

// 두 개의 그리드를 생성합니다.
createGrid('grid-container-1', 5, 6);
createGrid('grid-container-2', 3, 3);

// 예시: 첫 번째 그리드의 (1, 2) 위치의 네모를 녹색으로 변경
changeColor('grid-container-1', 1, 2, 'white');
changeColor('grid-container-1', 2, 2, 'white');
changeColor('grid-container-1', 4, 4, 'gray');
changeColor('grid-container-1', 2, 1, 'white');
changeColor('grid-container-1', 2, 3, 'white');
hideBox('grid-container-1', 1, 2);
