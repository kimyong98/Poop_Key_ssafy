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

                <Link to="/create_day" className="link"> 
                {/* 아직 탈 링크가 없음 이럴 때 #x를 하는듯 */}
                 Day 추가
                </Link>

            </div>

        </div>
    );
}