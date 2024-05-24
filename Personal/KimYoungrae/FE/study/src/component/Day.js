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

    // const [words, setWords] = useState([]);

    const words = useFetch(`http://localhost:3001/words?day=${day}`);


    // useEffect(()=>{
    //     fetch(`http://localhost:3001/words?day=${day}`)
    //     // 이러면 promise형태로 반환함 async/await에서 말하는 그거 맞음
    //     .then(res=>{
    //         // fetch가 되면 == .then
    //         return res.json()
    //         // 여기서 res는 http응답이고 실제 json은 아니라 .json()을 해야함
    //         //  이러면 json으로 변환되고, promise로 반환된다. (이게 뭔소리지??)
    //     })
    //     .then(data=>{
    //         // 위에걸 완료했다면
    //         setWords(data)
    //     })
    // }, [day]);

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





