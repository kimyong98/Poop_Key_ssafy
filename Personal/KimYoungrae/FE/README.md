
## 컴포넌트
컴포넌트는 component라는 폴더 아래에 만드는게 일반적

```javascript
function 함수명() {

    return (태그들);
}

export default 함수명

import 함수명 from '경로';
```

리턴시 괄호로

export defualt 함수명
 
<br>

## 이벤트

```javascript
function 함수명() {
    function 이벤트함수1() {
	console.log(...);
    }

    function 이벤트함수2(e) {
        console.log(e.target.value);
	...
	return ...
    }

    

    return (
	<button onClick={이벤트함수1}>sdsf</button> // 함수 자체를 실행
	<button onClick={이벤트함수2()}>sdsf</button> // 함수의 리턴값을 줌
    <button 
        onClick={()=>{
            console.log(30);
        }}
    >
    <input type="text" onChange={이벤트함수명2} />
    );
}




export default 함수명

import 함수명 from '경로';
```

이벤트에 함수를 넣을 시 
함수명 과 함수명()은 다른 의미를 가짐

input에서 값 가져오려면 e.target.value

<br>

## CSS 
```javascript
css파일명 : 컴포넌트이름.module.css

import styles from "./css파일명";


<태그 className={styles.box}>ㅁㄴㅇ</태그>
```

컴포넌트 아래에 컴포넌트.js와 컴포넌트.module.css를 같이 둔다.
이떄, 컴포넌트 숫자가 많아지면 component폴더 아래에 전용 폴더를 하나 더 생성한다.









## 반복시<ul><li> 처럼 반복시 index대신 key를 사용해야하는 이유
순서를 보장하기 어려운 경우를 위해 사용한다.

어떠한 객체에 포커싱을 해야하는데 고칠때마다 순서가 바뀌면 포커싱이 index를 기준으로 된다.

그리고 key를 써야 반복되는 요소를 전부 리 렌더링을 막을 수 있다.

즉, index방식은 자원낭비임

즉, 순서나 내용이 변화되는 상황이라면 요소를 특정하는게 가능한 key를 이용하자

## map사용방법
```javascript

const [list, setList] = useState({
    {id: 1, name: "Mike"},
    {id: 2, name: "Any"},
    {id: 3, name: "Jane"},
    {id: 4, name: "KIM"},
})




<div>
    {list.map((user) => (
        <div key={user.id}
        type='text'
        placeholder={user.name}
        />
    ))}

<div>
```










## App에서 컴포넌트들 사용하려면 상위에 타그 암거나 있어야함 <div>라거나





## react-router-dom
```javascript
import './App.css';
import Header from './component/Header'
import DayList from './component/DayList';
import Day from './component/Day';
import {BrowserRouter, Route, Switch} from 'react-router-dom';

function App() {
  return (
    <BrowserRouter>  
      <div className="App">
        <Header />
        <Switch>
          <DayList />
          <Day />
        </Switch>
      </div>
    </BrowserRouter>
  );
}

export default App;

```
리액트 라우트 돔을 사용하기 위해선

npm install react-router-dom@5.2.0   (현재 6버전이 있는데 실습에선 5버전)

import {BrowserRouter, Route, Switch} from 'react-router-dom';

전체를 <BrowserRouter>으로 감싸고

다음 페이지로 넘어가도 고정될 부분은 <Switch> 밖에

다음 페이지로 넘어가면 바뀔 부분은 <Switch> 안에다 넣어준다.
이러면 url에 따라 각각 바뀐다.





```javascript
function App() {
  return (
    <BrowserRouter>
      <div className="App">
        <Header />
        <Switch>
          <Route exact path="/">
            <DayList />
          </Route>
          
          <Route exact path="/day">
            <Day />
          </Route>
        </Switch>
      </div>
    </BrowserRouter>
    
  );
}

export default App;
```

그런 다음에 <Route path="/">로 각 링크마다 랜더링될 컴포넌트를 삽입한다.

다만 이렇게만 하면 문제가 생긴다.

"/day"에도 "/"가 포함되어서 "/"를 보고 이거네 하고 <DayList />를 보여준다.

따라서 extact를 추가해야함


```javascript
import {Link} from "react-router-dom";
import dummy from '../DB/data.json'

export default function DayList() {
    console.log(dummy);
    return (
        
        <ul className="list_day">
            {dummy.days.map(item =>(
                <li key={item.id}>
                    <Link to="./day">Day {item.day}</Link>
                </li>
            ))}
            
        </ul>

    );
}
```
import {Link} from "react-router-dom";를 해주고


이제 각 Day를 클릭하면 그 Day에 맞는 링크로 들어가야함.
이때 , html의 경우엔 a태그를 사용하지만
리액트에선 <Link to="/day">를 한다.

아직은 각 Day를 눌러도 하나의 날로만 간다.

(여기서 강의 와는 다른 이슈가 존재한다.)
index.js를 https://github.com/coding-angma/voca/blob/master/src/index.js 로 바꾸자



```javascript
import {Link} from "react-router-dom";
import dummy from '../DB/data.json'

export default function DayList() {
    console.log(dummy);
    return (
        
        <ul className="list_day">
            {dummy.days.map(item =>(
                <li key={item.id}>
                    <Link to={`/day/${item.day}`}>Day {item.day}</Link>
                </li>
            ))}
            
        </ul>

    );
}

```

각 day를 누르면 url이 /day/1 이런식으로 된다.
<Link to={`/day/${item.day}`}>Day {item.day}</Link>이다.

물론 아직 해당 day에 맞는걸로 넘어가진 않는다.

App.js를 수정하자


```javascript

import './App.css';
import Header from './component/Header'
import DayList from './component/DayList';
import Day from './component/Day';
import {BrowserRouter, Route, Switch} from 'react-router-dom';


function App() {
  return (
    <BrowserRouter>
      <div className="App">
        <Header />
        <Switch>
          <Route exact path="/">
            <DayList />
          </Route>
          
          <Route path="/day/:day">
            <Day />
          </Route>
        </Switch>
      </div>
    </BrowserRouter>
    
  );
}

export default App;
```




<Route path="/day/:day"> 이렇게 /:day를 추가하면 다이나믹하게 라우팅이 가능하다.

계속해서 Day.js 컴포넌트로 간다

Url에 포함된 값을 가져오기 위해선 

```javascript
import dummy from '../DB/data.json'
import { useParams } from 'react-router-dom';

export default function Day() {
    
    


    const a = useParams();
    console.log(a); // /day/2이면 day:'2'가 콘솔에 나옴 => 문자임에 조심하라!
    // 만약 <Route path="/day/:day">에서 :id로 바꾸면 id:'2'가 콘솔에 나온다.

    //console.log(wordList);
    const day = Number(a.day);

    const wordList = dummy.words.filter(word=>
        Number(word.day)===day
    );
    
    return <>
    <h2>Day {day}</h2>
    <table>
        <tbody>
            {wordList.map(word=>(
                <tr key={word.id}>
                    <td>
                        {word.eng}
                    </td>
                    <td>
                        {word.kor}
                    </td>
                </tr>
            ))}

        </tbody>
    </table>
    </>
}
```

import { useParams } from 'react-router-dom'; 라이브러리를 추가하고

링크에 있는 param (= :id 같은거)를 사용하기 위해 const a = useParams();를 한다.
a는 object형태이며, 숫자가 아닌 문자임에 주의해야한다.

이제 링크에 http://localhost:3000/day/askfnogn와 같은 이상한 params를 주면
아무것도 안 나온다.

이상한걸 넣었을 때를 위한 페이지(컴포넌트)를 만들자. EmptyPage.js

```javascript
// EmptyPage.js
import {Link} from 'react-router-dom'

export default function EmptyPage() {

    return <>
        <h2>잘못된 접근입니당!</h2>
        <Link to='/'>돌아가깅!</Link>
    </>;
}






// App.js

import './App.css';
import Header from './component/Header'
import DayList from './component/DayList';
import Day from './component/Day';
import EmptyPage from './component/EmptyPage';
import {BrowserRouter, Route, Switch} from 'react-router-dom';


function App() {
  return (
    <BrowserRouter>
      <div className="App">
        <Header />
        <Switch>
          <Route exact path="/">
            <DayList />
          </Route>
          
          <Route path="/day/:day">
            <Day />
          </Route>

          <Route> 
            {/* Route안에 path가 없으면 가장 조건으로 접근함...  주의! 이건 항상 가장 마지막에 해야함*/}
            <EmptyPage />
          </Route>
        </Switch>
      </div>
    </BrowserRouter>
    
  );
}

export default App;

```

주소창에 http://localhost:3000/asdklfnlasdfnk을 해보자
EmptyPage컴포넌트가 뜰거다.


## 이번엔 Day 아무거나 들어가면 뜻을 숨기거나 보이게 하는걸 만들어보자


```javascript
import dummy from '../DB/data.json'
import { useParams } from 'react-router-dom';

export default function Day() {
    
    


    const a = useParams();
    console.log(a); // /day/2이면 day:'2'가 콘솔에 나옴 => 문자임에 조심하라!
    // 만약 <Route path="/day/:day">에서 :id로 바꾸면 id:'2'가 콘솔에 나온다.

    //console.log(wordList);
    const day = Number(a.day);

    const wordList = dummy.words.filter(word=>
        Number(word.day)===day
    );
    
    return <>
    <h2>Day {day}</h2>
    <table>
        <tbody>
            {wordList.map(word=>(
                <tr key={word.id}>
                    <td>
                        <input type="checkbox"></input>
                    </td>
                    <td>
                        {word.eng}
                    </td>
                    <td>
                        {word.kor}
                    </td>
                    <td>
                        <button>뜻 보기</button>
                        <button class="btn_del">삭제</button>
                    </td>
                </tr>
            ))}

        </tbody>
    </table>
    </>
}
```

일단 Day 컴포넌트에 체크박스와 버튼 두 개를 추가한다.

각 단어마다 뜻을 보거나 가리기 위해서 컴포넌트로 따로 빼야한다.



```javascript
// Day.js
import dummy from '../DB/data.json'
import { useParams } from 'react-router-dom';
import Word from './Word'
export default function Day() {
    
    


    const a = useParams();
    console.log(a); // /day/2이면 day:'2'가 콘솔에 나옴 => 문자임에 조심하라!
    // 만약 <Route path="/day/:day">에서 :id로 바꾸면 id:'2'가 콘솔에 나온다.

    //console.log(wordList);
    const day = Number(a.day);

    const wordList = dummy.words.filter(word=>
        Number(word.day)===day
    );
    
    return <>
    <h2>Day {day}</h2>
    <table>
        <tbody>
            {wordList.map(word=>(
                <Word word={word} key={word.id}/>
            ))}

        </tbody>
    </table>
    </>
}






// Word.js
export default function Word(props) {

    return <>
    <tr key={props.word.id}>
                    <td>
                        <input type="checkbox"></input>
                    </td>
                    <td>
                        {props.word.eng}
                    </td>
                    <td>
                        {props.word.kor}
                    </td>
                    <td>
                        <button>뜻 보기</button>
                        <button class="btn_del">삭제</button>
                    </td>
                </tr>
    </>;
}

```

word를 props로 가져옴

그리고 보였다 사라졌다 해야하니 상태값 state를 사용해야함.


```javascript
import { useState } from "react";

export default function Word(props) {
    const [isShow, setIsShow] = useState(false);




    return <>
    <tr key={props.word.id}>
                    <td>
                        <input type="checkbox"></input>
                    </td>
                    <td>
                        {props.word.eng}
                    </td>
                    <td>
                        {isShow && props.word.kor}
                        {/* 이거 처음본다. true/false인 isShow와 &&를 한다니! */}
                    </td>
                    <td>
                        <button>뜻 보기</button>
                        <button class="btn_del">삭제</button>
                    </td>
                </tr>
    </>;
}
```

{isShow && props.word.kor} 에 익숙해져야 할듯 하다.
이제 사이트로 다시 가보면 한글 뜻이 사라져 잇다. (isShow가 false니까!!)


이제 뜻 보기 버튼을 누르면 보이게 해야함.

토글 함수를 하나 만들고 setIsShow를 이용하자!

```javascript
import { useState } from "react";

export default function Word(props) {
    const [isShow, setIsShow] = useState(false);

    function toggleShow() {
        setIsShow(!isShow);
    }

    return <>
    <tr key={props.word.id}>
                    <td>
                        <input type="checkbox"></input>
                    </td>
                    <td>
                        {props.word.eng}
                    </td>
                    <td>
                        {isShow && props.word.kor}
                        {/* 이거 처음본다. true/false인 isShow와 &&를 한다니! */}
                    </td>
                    <td>
                        <button onClick={toggleShow}>뜻 {isShow ? '숨기기' : '보이기'}</button>
                        {/* 이벤트 onClick 꼭 기억하자 */}
                        {/* isShow가 true면 숨기기로 나오겠다. */}
                        <button class="btn_del">삭제</button>
                    </td>
                </tr>
    </>;
}

```

{isShow && props.word.kor} 랑 <button onClick={toggleShow}>뜻 {isShow ? '숨기기' : '보이기'}</button> 잘 이해하자!!!



이제 외운거면 -- 쳐서 

외원거랑 안 외운거를 구분할거다

(강의랑 생긴게 달라서 강사님 index.css를 가져옴  https://github.com/coding-angma/voca/blob/master/src/index.css )



```javascript
import { useState } from "react";

export default function Word(props) {
    const [isShow, setIsShow] = useState(false);
    const [isDone, setIsDone] = useState(props.word.isDone);
    // state는 해당 컴포넌트에 있는걸 바꿔야지 props는 건드는거 아님 ㅎㅎ

    function toggleShow() {
        setIsShow(!isShow);
    }

    function toggleDone() {
        setIsDone(!isDone);
    }

    return <>
    <tr className={isDone ? "off" : ""} key={props.word.id}>
        <td>
            <input type="checkbox" checked={isDone}
            onChange={toggleDone}></input>
        </td>
        <td>
            {props.word.eng}
        </td>
        <td>
            {isShow && props.word.kor}
            {/* 이거 처음본다. true/false인 isShow와 &&를 한다니! */}
        </td>
        <td>
            <button onClick={toggleShow}>뜻 {isShow ? '숨기기' : '보이기'}</button>
            {/* 이벤트 onClick 꼭 기억하자 */}
            {/* isShow가 true면 숨기기로 나오겠다. */}
            <button className="btn_del">삭제</button>
        </td>
    </tr>
    </>;
}
```

state로 isDone을 추가하였다. 이때 외부의 데이타인 props를 건드릴 수는 없으니
isDone을 자체적 state로 하자. 
props.word.isDone은 맨 처음만 써야한다는 말!

그리고 리액트는 class보단 className을 원함





## 이제 사용자 입장에서 단어장에 데이터를 추가하거나 삭제하기 위한 DB를 구축하고 API를 만들어야 한다.  json 서버를 이용해서 RESTful API를 해보자

여기서 REST API가 무엇이냐?

=> Create POST / Read GET / Update PUT / Delete DELETE

npm install -g json-server

여기서 -g는 글로발이닫.

json-server --watch ./src/DB/data.jspn --port 3001

http://localhost:3001/days를 가보자
http://localhost:3001/words를 가보자

http://localhost:3001/words/2를 하면 id가 2인 녀석만 나온다.

http://localhost:3001/words?day=1를하면 day가 1인 녀석만 나온다.




## 이제 REST API로 할거니까 기존의 코드를 바꿔보자 API를 이용해서 데이터를 가져오는 방식으로 useEffect, fetch()

useEffect()의 첫번째 인자는 함수이다. 보통 화살표 함수를 많이 쓴다.

useEffect()는 랜더링 결과가 실제 DOM에 발리는 직후 실행된다.
마찬가지로 컴포넌트가 사라지기 직전에도 실행된다.


DayList.js로 가자

```javascript
import { useState, useEffect } from "react";
import {Link} from "react-router-dom";
// import dummy from '../DB/data.json'

export default function DayList() {
    const [days, setDays] = useState([])
    const [count, setCount] = useState(0);

    function onClick() {
        setCount(count + 1);
    }

    useEffect(()=>{
        console.log("count change");
    })

    return <>
        
        <ul className="list_day">
            {days.map(item =>(
                <li key={item.id}>
                    <Link to={`/day/${item.day}`}>Day {item.day}</Link>
                </li>
            ))}
            
        </ul>
        <button onClick={onClick}>{count}</button>

     </>;
}

```

버튼을 누르면 count가 바뀌면서 다시 랜더링이 된다.
따라서 버튼을 누르면 새로 랜더링 되는 거고 useEffect()가 호출되는 것이다.


근데 이러면 안 좋은 점이 있음.

state가 바뀔때마다 useEffect()가 된다는거임...

내가 원하는 특정 state가 바뀌는 상황에서만 call되게 해보자.

useEffect()의 두번째 인자로 배열을 넣자.

배열안에는 내가 useEffect가 지켜볼 state를 넣는다.

이를 의존성 배열이라 한다.


```javascript
import { useState, useEffect } from "react";
import {Link} from "react-router-dom";
// import dummy from '../DB/data.json'

export default function DayList() {
    const [days, setDays] = useState([])
    const [count, setCount] = useState(0);

    function onClick() {
        setCount(count + 1);
    }

    function onClick2() {
        setDays([
            ...days,
            {
                id: Math.random(),
                day: 1,
            }
        ]);
    }

    useEffect(()=>{
        console.log("count change");
    }, [count])

    return <>
        
        <ul className="list_day">
            {days.map(item =>(
                <li key={item.id}>
                    <Link to={`/day/${item.day}`}>Day {item.day}</Link>
                </li>
            ))}
            
        </ul>
        <button onClick={onClick}>{count}</button>
        <button onClick={onClick2}>데이</button>

     </>;
}
```

근데 현재 우리가 useEffect()를 사용하는 목적은 API호출임

랜더링이 되고 최초의 한번만 API를 호출해서 데이타를 가져온다. 가 목표이다.

의존성 배열을 빈 배열로 하자

이러면 최초의 한번만 useEffect()가 실행되서

콘솔창에는 딱 한번만 Count change가 뜨고, button들을 눌러도 call되지 않는다.

이제 진짜 API를 호출되는 코드로 바꿔보자.

API 비통기 호출을 위해 fetch()를 사용합니다.

```javascript
import { useState, useEffect } from "react";
import {Link} from "react-router-dom";
// import dummy from '../DB/data.json'

export default function DayList() {
    const [days, setDays] = useState([])



    useEffect(()=>{
        fetch('http://localhost:3001/days')
        // 이러면 promise형태로 반환함 async/await에서 말하는 그거 맞음
        .then(res=>{
            // fetch가 되면 == .then
            return res.json()
            // 여기서 res는 http응답이고 실제 json은 아니라 .json()을 해야함
            //  이러면 json으로 변환되고, promise로 반환된다. (이게 뭔소리지??)
        })
        .then(data=>{
            // 위에걸 완료했다면
            setDays(data)
        })
    }, [])

    return <>
        
        <ul className="list_day">
            {days.map(item =>(
                <li key={item.id}>
                    <Link to={`/day/${item.day}`}>Day {item.day}</Link>
                </li>
            ))}
            
        </ul>


     </>;
}

```

이제 http://localhost:3000/ 로 가면

day들이 잘 나온다. 디버깅 팁으로 F12에서 Network를 누르고 Fetch/XHR을 눌러서 200 OK인걸 확인해보자.





이번엔 Day.js로 가보자

```javascript
// 바꾸기 전

import dummy from '../DB/data.json'
import { useParams } from 'react-router-dom';
import Word from './Word'
export default function Day() {
    
    


    const a = useParams();
    console.log(a); // /day/2이면 day:'2'가 콘솔에 나옴 => 문자임에 조심하라!
    // 만약 <Route path="/day/:day">에서 :id로 바꾸면 id:'2'가 콘솔에 나온다.

    //console.log(wordList);
    const day = Number(a.day);

    const wordList = dummy.words.filter(word=>
        Number(word.day)===day
    );
    
    return <>
    <h2>Day {day}</h2>
    <table>
        <tbody>
            {wordList.map(word=>(
                <Word word={word} key={word.id}/>
            ))}

        </tbody>
    </table>
    </>
}

```

```javascript
//import dummy from '../DB/data.json'
import { useParams} from 'react-router-dom';
import { useEffect , useState } from 'react';
import Word from './Word'
export default function Day() {
    
    


    const a = useParams();
    //console.log(a); // /day/2이면 day:'2'가 콘솔에 나옴 => 문자임에 조심하라!
    // 만약 <Route path="/day/:day">에서 :id로 바꾸면 id:'2'가 콘솔에 나온다.

    //console.log(wordList);
    const day = Number(a.day);

    const [words, setWords] = useState([]);

    useEffect(()=>{
        fetch(`http://localhost:3001/words?day=${day}`)
        // 이러면 promise형태로 반환함 async/await에서 말하는 그거 맞음
        .then(res=>{
            // fetch가 되면 == .then
            return res.json()
            // 여기서 res는 http응답이고 실제 json은 아니라 .json()을 해야함
            //  이러면 json으로 변환되고, promise로 반환된다. (이게 뭔소리지??)
        })
        .then(data=>{
            // 위에걸 완료했다면
            setWords(data)
        })
    }, [day]);

    // const wordList = dummy.words.filter(word=>Number(word.day)===day);
    
    return <>
    <h2>Day {day}</h2>
    <table>
        <tbody>
            {words.map(word=>(
                <Word word={word} key={word.id}/>
            ))}

        </tbody>
    </table>
    </>
}
```

DayList.js와 유사하나 url의 day값을 가져오기 위한게 살짝 다름

여기서 의존성배열을 day를 준 이유는 useEffect()내부에서 사용한 변수는 의존성 배열에 추가하는게 좋다는 경고문이 생기기 때문이다.



## 커스텀 훅
json-server --watch ./src/DB/data.json --port 3001 다시 하자

```javascript
// 수정 전 Day

//import dummy from '../DB/data.json'
import { useParams} from 'react-router-dom';
import { useEffect , useState } from 'react';
import Word from './Word'
export default function Day() {
    
    


    const a = useParams();
    //console.log(a); // /day/2이면 day:'2'가 콘솔에 나옴 => 문자임에 조심하라!
    // 만약 <Route path="/day/:day">에서 :id로 바꾸면 id:'2'가 콘솔에 나온다.

    //console.log(wordList);
    const day = Number(a.day);

    const [words, setWords] = useState([]);

    useEffect(()=>{
        fetch(`http://localhost:3001/words?day=${day}`)
        // 이러면 promise형태로 반환함 async/await에서 말하는 그거 맞음
        .then(res=>{
            // fetch가 되면 == .then
            return res.json()
            // 여기서 res는 http응답이고 실제 json은 아니라 .json()을 해야함
            //  이러면 json으로 변환되고, promise로 반환된다. (이게 뭔소리지??)
        })
        .then(data=>{
            // 위에걸 완료했다면
            setWords(data)
        })
    }, [day]);

    // const wordList = dummy.words.filter(word=>Number(word.day)===day);
    
    return <>
    <h2>Day {day}</h2>
    <table>
        <tbody>
            {words.map(word=>(
                <Word word={word} key={word.id}/>
            ))}

        </tbody>
    </table>
    </>
}
```

useEffect()부분을 사용자 Hook으로 만들어 url만 넣으면 되는걸로 바꾸겠다.

src/hooks폴더를 만들고, useFetch.js를 만든다.


```javascript

// useFetch.js
import { useEffect, useState } from "react"

export default function useFetch(url) {
    

    const [data, setData] = useState([]);

    useEffect(()=>{
        fetch(url)
        // 이러면 promise형태로 반환함 async/await에서 말하는 그거 맞음
        .then(res=>{
            // fetch가 되면 == .then
            return res.json()
            // 여기서 res는 http응답이고 실제 json은 아니라 .json()을 해야함
            //  이러면 json으로 변환되고, promise로 반환된다. (이게 뭔소리지??)
        })
        .then(data=>{
            // 위에걸 완료했다면
            setData(data)
        })
    }, [url]);

    return data;
}

```


```javascript

// DayList.js
import { useState, useEffect } from "react";
import {Link} from "react-router-dom";
import useFetch from "../hooks/useFetch";
// import dummy from '../DB/data.json'

export default function DayList() {
    const days = useFetch("http://localhost:3001/days");
    
    console.log("days object");
    console.log(days);
    
 
    return <>
        
        <ul className="list_day">
            {days.map(item =>(
                <li key={item.id}>
                    <Link to={`/day/${item.day}`}>Day {item.day}</Link>
                </li>
            ))}
            
        </ul>


     </>;
}

```


```javascript
//import dummy from '../DB/data.json'
import { useParams} from 'react-router-dom';
import { useEffect , useState } from 'react';
import useFetch from '../hooks/useFetch';
import Word from './Word'
export default function Day() {
    
    


    const a = useParams();
    //console.log(a); // /day/2이면 day:'2'가 콘솔에 나옴 => 문자임에 조심하라!
    // 만약 <Route path="/day/:day">에서 :id로 바꾸면 id:'2'가 콘솔에 나온다.

    //console.log(wordList);
    const day = Number(a.day);


    const words = useFetch(`http://localhost:3001/words?day=${day}`);


    return <>
    <h2>Day {day}</h2>
    <table>
        <tbody>
            {words.map(word=>(
                <Word word={word} key={word.id}/>
            ))}

        </tbody>
    </table>
    </>
}
```





## CRUD에서 PUT / DELETE

현재 DAY1~3암거나 들어가서 
왼쪽 외웠는지 안 외웠는지를 체크하고

새로 고침하면 state가 날라가서 기억을 못함.

따라서 DB에 저장하는게 좋음

이걸 해보겠다.

즉, data.json의 words에서 각 isDone의 값을 수정하겠음.

```javascript
// Word.js    바꾸기전

import { useState } from "react";

export default function Word(props) {
    const [isShow, setIsShow] = useState(false);
    const [isDone, setIsDone] = useState(props.word.isDone);
    // state는 해당 컴포넌트에 있는걸 바꿔야지 props는 건드는거 아님 ㅎㅎ

    function toggleShow() {
        setIsShow(!isShow);
    }

    function toggleDone() {
        setIsDone(!isDone);
    }

    return <>
    <tr className={isDone ? "off" : ""} key={props.word.id}>
        <td>
            <input type="checkbox" checked={isDone}
            onChange={toggleDone}></input>
        </td>
        <td>
            {props.word.eng}
        </td>
        <td>
            {isShow && props.word.kor}
            {/* 이거 처음본다. true/false인 isShow와 &&를 한다니! */}
        </td>
        <td>
            <button onClick={toggleShow}>뜻 {isShow ? '숨기기' : '보이기'}</button>
            {/* 이벤트 onClick 꼭 기억하자 */}
            {/* isShow가 true면 숨기기로 나오겠다. */}
            <button className="btn_del">삭제</button>
        </td>
    </tr>
    </>;
}

```



```javascript
import { useState, useEffect } from "react";

export default function Word(props) {
    const [isShow, setIsShow] = useState(false);
    const [isDone, setIsDone] = useState(props.word.isDone);
    // state는 해당 컴포넌트에 있는걸 바꿔야지 props는 건드는거 아님 ㅎㅎ

    function toggleShow() {
        setIsShow(!isShow);
    }

    function toggleDone() { // 여길 바꾸겠음
        // setIsDone(!isDone);
        fetch(`http://localhost:3001/words/${props.word.id}`, {
            method : 'PUT',
            headers : {
                'Content-Type' : 'application/json',
                // Content-Type은 보내는 리소스의 타입을 의미함.
                // 타입을 json으로 보내주겠음.
                // 타입은 이미지 json 등등 다양함
            },
            body : JSON.stringify({
                ...props.word,
                isDone : !isDone,
            }),
        })
        .then(res=>{
            if(res.ok) {
                setIsDone(!isDone);
            }
        });
    }

    return <>
    <tr className={isDone ? "off" : ""} key={props.word.id}>
        <td>
            <input type="checkbox" checked={isDone}
            onChange={toggleDone}></input>
        </td>
        <td>
            {props.word.eng}
        </td>
        <td>
            {isShow && props.word.kor}
            {/* 이거 처음본다. true/false인 isShow와 &&를 한다니! */}
        </td>
        <td>
            <button onClick={toggleShow}>뜻 {isShow ? '숨기기' : '보이기'}</button>
            {/* 이벤트 onClick 꼭 기억하자 */}
            {/* isShow가 true면 숨기기로 나오겠다. */}
            <button className="btn_del">삭제</button>
        </td>
    </tr>
    </>;
}

```

### 잠깐! 깨달은게 있다!
나는 fetch()가 useEffect()의 일부분인줄 알았으나, js의 기본 메소드로
axios와 비슷하다.

다만, axios는 가져와야하고 axios쪽이 확장성이 더 높다.

그러니 axios로 해야겠다.

이제 isDone을 json에다 put (UPDATE) 해서 새로고침해도 반영이 된다.

```javascript
function toggleDone() { // 여길 바꾸겠음
    const [isDone, setIsDone] = useState(props.word.isDone);

    fetch(`http://localhost:3001/words/${props.word.id}`, {
        method : 'PUT',
        headers : {
            'Content-Type' : 'application/json',
            // Content-Type은 보내는 리소스의 타입을 의미함.
            // 타입을 json으로 보내주겠음.
            // 타입은 이미지 json 등등 다양함
        },
        body : JSON.stringify({
            ...props.word,
            isDone : !isDone,
        }),
    })
    .then(res=>{
        if(res.ok) {
            setIsDone(!isDone);
        }
    });
}
```

body에 데이터를 실어 보낸다.

이때, json 형태로 보내기 위해 
JSON.stringfy({오브젝트});해야함



UPDATE 처럼 DELETE도 해보자


```javascript
import { useState, useEffect } from "react";

export default function Word(props) {

    // word를 state로 하겠음
    // 왜냐하면 DELETE를 했을 때, 새로 랜더링해야하는데 state로 하면 편하니까.
    const [word, setWord] = useState(props.word);

    const [isShow, setIsShow] = useState(false);
    const [isDone, setIsDone] = useState(props.word.isDone);
    // state는 해당 컴포넌트에 있는걸 바꿔야지 props는 건드는거 아님 ㅎㅎ

    function toggleShow() {
        setIsShow(!isShow);
    }

    function toggleDone() { // 여길 바꾸겠음
        // setIsDone(!isDone);
        fetch(`http://localhost:3001/words/${props.word.id}`, {
            method : 'PUT',
            headers : {
                'Content-Type' : 'application/json',
                // Content-Type은 보내는 리소스의 타입을 의미함.
                // 타입을 json으로 보내주겠음.
                // 타입은 이미지 json 등등 다양함
            },
            body : JSON.stringify({
                ...props.word,
                isDone : !isDone,
            }),
        })
        .then(res=>{
            if(res.ok) {
                setIsDone(!isDone);
            }
        });
    }

    function del() {
        if(window.confirm('진짜 삭제할꺼야?')) {
            fetch(`http://localhost:3001/words/${props.word.id}`, {
                method : 'DELETE',
            }).then(res=>{
                // .then 이후는 새로 랜더링하기 위한거임
                if(res.ok) {
                    setWord({id:0});
                }
            })

        }
    }

    if (word.id === 0) { // id가 0이면 null을 하는데 null이면 아래 return을 안 해서 아무것도 안뜸.
        return null;
    }

    return <>
    <tr className={isDone ? "off" : ""} key={props.word.id}>
        <td>
            <input type="checkbox" checked={isDone}
            onChange={toggleDone}></input>
        </td>
        <td>
            {props.word.eng}
        </td>
        <td>
            {isShow && props.word.kor}
            {/* 이거 처음본다. true/false인 isShow와 &&를 한다니! */}
        </td>
        <td>
            <button onClick={toggleShow}>뜻 {isShow ? '숨기기' : '보이기'}</button>
            {/* 이벤트 onClick 꼭 기억하자 */}
            {/* isShow가 true면 숨기기로 나오겠다. */}
            <button className="btn_del" onClick={del}>삭제</button>
        </td>
    </tr>
    </>;
}
```

DB에서만 DELETE를 하면 바로 해당 페이지에 적용이 안 된다.

따라서 word 객체를 state로 만들고 word 객체의 id를 0으로 만든다.

id가 0이면 return null로 한다. 이러면 아래쪽 return 문 대신에 null이 리턴되는데

null을 받으면 아무것도 안 뜬다.


## 이제 마지막 CRUD의 POST=CREATAE를 해보자!
컴포넌트로 CreateWord.js를 만들겠음


```javascript
// App.js

import './App.css';
import Header from './component/Header'
import DayList from './component/DayList';
import Day from './component/Day';
import EmptyPage from './component/EmptyPage';
import CreateWord from './component/CreateWord';
import {BrowserRouter, Route, Switch} from 'react-router-dom';


function App() {
  return (
    <BrowserRouter>
      <div className="App">
        <Header />
        <Switch>
          <Route exact path="/">
            <DayList />
          </Route>
          
          <Route path="/day/:day">
            <Day />
          </Route>

          <Route path="/create_word">
            <CreateWord />
          </Route>

          <Route> 
            {/* Route안에 path가 없으면 가장 조건으로 접근함...  주의! 이건 항상 가장 마지막에 해야함*/}
            <EmptyPage />
          </Route>
        </Switch>
      </div>
    </BrowserRouter>
    
  );
}

export default App;

```

App.js에선 <Route>로 라우트를 하나 추가하고

해당 라우트(url)로 가면 CreateWord 컴포넌트가 Switch내에 랜더링 되게 함.

마찬가지로 화면 윗단인 Header 컴포넌트도 수정하자.

```javascript
// Header.js

import {Link} from 'react-router-dom'
export default function Header() {
    return (
        <div className='header'>
            <h1>
                <Link to="/"> 토익 영단어 </Link>
            </h1>
            <div className="menu">
                <Link to="/create_word" className="link">   
                 단어추가
                </Link>

                <Link to="#x" className="link"> 
                {/* 아직 탈 링크가 없음 이럴 때 #x를 하는듯 */}
                 Day 추가
                </Link>

            </div>

        </div>
    );
}
```
여기서 중요한건 아직 링크가 없는 경우 #x로 한다는 것.

<Link>는 <a>와 비슷하다는걸 항상 명심하자.




```javascript
// CreateWords.js 중간과정

import useFetch from "../hooks/useFetch"
import { useRef } from "react";

export default function CreateWord() {
    const days = useFetch(`http://localhost:3001/days`);
    

    function onSubmit(e) {
        e.preventDefault();
        // form 내부의 버튼을 눌렀을 때 새로고침 막는거

        // 이제 각 ref에 잘 담기는지 확인하자
        console.log(engRef.current.value)
        console.log(korRef.current.value)
        console.log(dayRef.current.value)
        // 잘 나오는걸 확인 할 수 있다.

    } 

    const engRef = useRef(null);
    const korRef = useRef(null);
    const dayRef = useRef(null);
    // useRef란?
    // DOM에 접근하게 해준다 (querySelector 같은거)
    // 스크롤 위치를 확인하거나 포커를 주거나 할 때 사용함



    return <form onSubmit={onSubmit}>
        <div className="input_area">
            <label>Eng</label>
            {/* 각 입력부분마다 ref를 연결한다. */}
            <input type="text" placeholder="computer" ref={engRef}></input>
        </div>

        <div className="input_area">
            <label>Kor</label>
            <input type="text" placeholder="컴퓨터" ref={korRef}></input>
        </div>

        <div className="input_area">
            <label>Day</label>
            {/* select - option 기억하자 */}
            <select ref={dayRef}>
                {days.map(day=>(
                    <option key={day.id} value={day.day}>{day.day}</option>
                ))}
            </select>
            {/*  map(day=>())) =>다음에 ()이야!!! */}
        </div>
        <button>저장</button>
        {/*  */}
    </form>
}
```

너무 길어서 POST 직전에서 끊었다.

복습할 부분은 

select - option 구문

변수.map(반복변수=> () ) 

태그를 { }안에 넣을 수 있음.

<form> 태그안의 버튼은 submit 기능을 함.
또한 버튼을 누르면 form태그의 특성상 새로고침이 일어남.
이를 보기 싫어서 e.preventDefault();를 함수로 만들고 onSubmit={함수명} 하였음.

useRef는 querySelector랑 동일함.

다만 값에 접근하기 위해선 다음과 같이 해야함. 

engRef.current.value


<br><br>

이제 진짜로 단어를 추가해보자

일단 data.json에 word에 해당하는 필드가 무엇인지 확인하자.

id / day / eng / kor / isDone이 있다.

POST시 id는 자동적으로 다음 순서를 입력한다.

isDone은 초기값으로 false를 고정하곘음

따라서 넣을 땐 day / eng / kor / isDone을 넣고,
isDone의 값은 고정하자.


```javascript
// CreateWord.js

import useFetch from "../hooks/useFetch"
import { useRef } from "react";

export default function CreateWord() {
    const days = useFetch(`http://localhost:3001/days`);

    function onSubmit(e) {
        e.preventDefault();
        // form 내부의 버튼을 눌렀을 때 새로고침 막는거

        // 이제 각 ref에 잘 담기는지 확인하자
        console.log(engRef.current.value)
        console.log(korRef.current.value)
        console.log(dayRef.current.value)
        // 잘 나오는걸 확인 할 수 있다.


        // Word.js에서 PUT에 해당하는 부분을 긁어와 수정하겠다.
        // POST 코드
        fetch(`http://localhost:3001/words/`, {
            //  주소는 ${props.word.id} 제거 해 주자.
            method : 'POST',
            headers : {
                'Content-Type' : 'application/json',
                // Content-Type은 보내는 리소스의 타입을 의미함.
                // 타입을 json으로 보내주겠음.
                // 타입은 이미지 json 등등 다양함
            },
            body : JSON.stringify({
                day : dayRef.current.value,
                eng : engRef.current.value,
                kor : korRef.current.value,
                isDone : false,
            }),
        })
        .then(res=>{
            if(res.ok) {
                alert("단어 추가 완료!")
            }
        });
    }



    

    const engRef = useRef(null);
    const korRef = useRef(null);
    const dayRef = useRef(null);
    // useRef란?
    // DOM에 접근하게 해준다 (querySelector 같은거)
    // 스크롤 위치를 확인하거나 포커를 주거나 할 때 사용함



    return <form onSubmit={onSubmit}>
        <div className="input_area">
            <label>Eng</label>
            {/* 각 입력부분마다 ref를 연결한다. */}
            <input type="text" placeholder="computer" ref={engRef}></input>
        </div>

        <div className="input_area">
            <label>Kor</label>
            <input type="text" placeholder="컴퓨터" ref={korRef}></input>
        </div>

        <div className="input_area">
            <label>Day</label>
            {/* select - option 기억하자 */}
            <select ref={dayRef}>
                {days.map(day=>(
                    <option key={day.id} value={day.day}>{day.day}</option>
                ))}
            </select>
            {/*  map(day=>())) =>다음에 ()이야!!! */}
        </div>
        <button>저장</button>
        {/*  */}
    </form>
}
```

POST까지 해보았다.

form의 내용들을 채워넣고 버튼을 누르면 alert가 뜨고 실제로 data.json에 추가된다.

여기서 id값은 중복이 없는 임의의 값이 된다.

```javascript
body : JSON.stringify({
    day : dayRef.current.value,
    eng : engRef.current.value,
    kor : korRef.current.value,
    isDone : false,
}),
```
가 가장 중요한 부분




## 단어를 추가하면 그 날짜에 해당하는 페이지로 자동적으로 가서 보여주도록 하자. useHistory


```javascript
// CreateWord.js

import { useHistory } from "react-router-dom";
import useFetch from "../hooks/useFetch"
import { useRef } from "react";

export default function CreateWord() {
    const days = useFetch(`http://localhost:3001/days`);
    const history = useHistory();

    function onSubmit(e) {
        e.preventDefault();
        // form 내부의 버튼을 눌렀을 때 새로고침 막는거

        // 이제 각 ref에 잘 담기는지 확인하자
        console.log(engRef.current.value)
        console.log(korRef.current.value)
        console.log(dayRef.current.value)
        // 잘 나오는걸 확인 할 수 있다.


        // Word.js에서 PUT에 해당하는 부분을 긁어와 수정하겠다.
        // POST 코드
        fetch(`http://localhost:3001/words/`, {
            //  주소는 ${props.word.id} 제거 해 주자.
            method : 'POST',
            headers : {
                'Content-Type' : 'application/json',
                // Content-Type은 보내는 리소스의 타입을 의미함.
                // 타입을 json으로 보내주겠음.
                // 타입은 이미지 json 등등 다양함
            },
            body : JSON.stringify({
                day : dayRef.current.value,
                eng : engRef.current.value,
                kor : korRef.current.value,
                isDone : false,
            }),
        })
        .then(res=>{
            if(res.ok) {
                alert("단어 추가 완료!");
                // 해당 페이지로 이동 Link to와 유사하다.
                history.push(`/day/${dayRef.current.value}`);
            }
        });
    }



    

    const engRef = useRef(null);
    const korRef = useRef(null);
    const dayRef = useRef(null);
    // useRef란?
    // DOM에 접근하게 해준다 (querySelector 같은거)
    // 스크롤 위치를 확인하거나 포커를 주거나 할 때 사용함



    return <form onSubmit={onSubmit}>
        <div className="input_area">
            <label>Eng</label>
            {/* 각 입력부분마다 ref를 연결한다. */}
            <input type="text" placeholder="computer" ref={engRef}></input>
        </div>

        <div className="input_area">
            <label>Kor</label>
            <input type="text" placeholder="컴퓨터" ref={korRef}></input>
        </div>

        <div className="input_area">
            <label>Day</label>
            {/* select - option 기억하자 */}
            <select ref={dayRef}>
                {days.map(day=>(
                    <option key={day.id} value={day.day}>{day.day}</option>
                ))}
            </select>
            {/*  map(day=>())) =>다음에 ()이야!!! */}
        </div>
        <button>저장</button>
    </form>
}

```


여기서 중요한 부분은

```javascript
const history = useHistory();
```
와
```javascript
.then(res=>{
            if(res.ok) {
                alert("단어 추가 완료!");
                // 해당 페이지로 이동 Link to와 유사하다.
                history.push(`/day/${dayRef.current.value}`);
            }
        });
```
이다.

history.push(경로)를 하면 Link to 태그를 클릭한거처럼 작동한다.

해당 코드에선 POST후 응답이 ok면 자동적으로 한다.


## 마찬가지로 DAY도 늘려보자 DAY5 DAY6 ...
컴포넌트로 CreateDay.js를 만들자

```javascript
// CreateDay.js
import { useHistory } from "react-router-dom";
import useFetch from "../hooks/useFetch"

export default function CreateWord() {
    const days = useFetch(`http://localhost:3001/days`);
    const history = useHistory(); 
    function addDay() {
        fetch(`http://localhost:3001/days/`, {
            //  주소는 ${props.word.id} 제거 해 주자.
            method : 'POST',
            headers : {
                'Content-Type' : 'application/json',
                // Content-Type은 보내는 리소스의 타입을 의미함.
                // 타입을 json으로 보내주겠음.
                // 타입은 이미지 json 등등 다양함
            },
            body : JSON.stringify({
                day : days.length + 1,
            }),
        })
        .then(res=>{
            if(res.ok) {
                alert("단어 추가 완료!");
                // 해당 페이지로 이동 Link to와 유사하다.
                history.push(`/`);
            }
        });
    }

    return <>
        <div>현재 일수 : {days.length}일</div>
        <button onClick={addDay}>Day 추가</button>
    </>
}

```

여기서 새로웠던 부분은 days.length





## 마지막으로 SPA인 리액트는 로딩이 오래 걸린다. 로딩이면 로딩중인걸 표시하자.

네트워크 설정을 3G로 바꾸고 새로고침을 해보자

어디를 불러오느라 느린지 바로 알 수 있다.

보통 Fetch(= axios)하는 부분이 느리다.

그 부분만 return 으로 데이터 길이가 0(=로딩중)이니까


```javascript
import { useState, useEffect } from "react";
import {Link} from "react-router-dom";
import useFetch from "../hooks/useFetch";
// import dummy from '../DB/data.json'

export default function DayList() {
    const days = useFetch("http://localhost:3001/days");

    if(days.length === 0) {
        return <span>Now Loading...</span>
    }
    
   
    return <>
        
        <ul className="list_day">
            {days.map(item =>(
                <li key={item.id}>
                    <Link to={`/day/${item.day}`}>Day {item.day}</Link>
                </li>
            ))}
            
        </ul>


     </>;
}
```

```javascript
if(days.length === 0) {
    return <span>Now Loading...</span>
}
```
를 추가해서 길이가 0 == 데이터 로딩 중인걸 따로 처리했다.




마찬가지로 CreateWord에서도 추가 버튼을 마구 누르면 계속 들어가는데 
로딩중이라는 state를 하나 선언한다.

```javascript
import { useHistory } from "react-router-dom";
import useFetch from "../hooks/useFetch"
import { useRef, useState } from "react";

export default function CreateWord() {
    const days = useFetch(`http://localhost:3001/days`);
    const history = useHistory();
    const [isLoading, setIsLoading] = useState(false); 

    function onSubmit(e) {
        e.preventDefault();

        if(!isLoading) {
            setIsLoading(true);


            // form 내부의 버튼을 눌렀을 때 새로고침 막는거

            // 이제 각 ref에 잘 담기는지 확인하자
            console.log(engRef.current.value)
            console.log(korRef.current.value)
            console.log(dayRef.current.value)
            // 잘 나오는걸 확인 할 수 있다.


            // Word.js에서 PUT에 해당하는 부분을 긁어와 수정하겠다.
            // POST 코드
            fetch(`http://localhost:3001/words/`, {
                //  주소는 ${props.word.id} 제거 해 주자.
                method : 'POST',
                headers : {
                    'Content-Type' : 'application/json',
                    // Content-Type은 보내는 리소스의 타입을 의미함.
                    // 타입을 json으로 보내주겠음.
                    // 타입은 이미지 json 등등 다양함
                },
                body : JSON.stringify({
                    day : dayRef.current.value,
                    eng : engRef.current.value,
                    kor : korRef.current.value,
                    isDone : false,
                }),
            })
            .then(res=>{
                if(res.ok) {
                    alert("단어 추가 완료!");
                    // 해당 페이지로 이동 Link to와 유사하다.
                    history.push(`/day/${dayRef.current.value}`);
                    setIsLoading(false);
                }
            });
        }
    }



    

    const engRef = useRef(null);
    const korRef = useRef(null);
    const dayRef = useRef(null);
    // useRef란?
    // DOM에 접근하게 해준다 (querySelector 같은거)
    // 스크롤 위치를 확인하거나 포커를 주거나 할 때 사용함



    return <form onSubmit={onSubmit}>
        <div className="input_area">
            <label>Eng</label>
            {/* 각 입력부분마다 ref를 연결한다. */}
            <input type="text" placeholder="computer" ref={engRef}></input>
        </div>

        <div className="input_area">
            <label>Kor</label>
            <input type="text" placeholder="컴퓨터" ref={korRef}></input>
        </div>

        <div className="input_area">
            <label>Day</label>
            {/* select - option 기억하자 */}
            <select ref={dayRef}>
                {days.map(day=>(
                    <option key={day.id} value={day.day}>{day.day}</option>
                ))}
            </select>
            {/*  map(day=>())) =>다음에 ()이야!!! */}
        </div>
        <button>{isLoading ? "Now Saving..." : "저장"}</button>
    </form>
}
```

```javascript
const [isLoading, setIsLoading] = useState(false); 

if(!isLoading) {
    setIsLoading(true);
...

    setIsLoading(false);
}

<button>{isLoading ? "Now Saving..." : "저장"}</button>

```

이러한 것들을 추가하였다.


## 끝~







<br><br><br><br><br><br>
<br><br><br><br><br><br>










## 이제 CSS다. Tail Wind

아 저는 css 버리고 tailwind 씁니다 by 코딩애플 

소개 영상을 들었다.

대분류 클래스명
- Typography
- Spacing
- Colors
- Flex
- grid
- 반응형 조건문 (md:) 예시 : 윈도우 크기가 몇 이상일때만
- pseudo class 조건문 (hover:) 예시 : 해당 태그 마우스 위에 올리면 글이 나타남





근데 className에 하나하나 다 적기는 매우 귀찮음

그래서 index.css에 

@layer components {
    .className {
        @apply m-5 p-4 ... ;
    }
}


