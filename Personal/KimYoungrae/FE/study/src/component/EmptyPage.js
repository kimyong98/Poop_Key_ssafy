import {Link} from 'react-router-dom'

export default function EmptyPage() {

    return <>
        <h2>잘못된 접근입니당!</h2>
        <Link to='/'>돌아가깅!</Link>
    </>;
}