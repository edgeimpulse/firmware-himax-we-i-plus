const jpegjs = require('jpeg-js');

const snapshot = Buffer.from('qaisrq+ssK2ur7CsrqqusrGxsa6ws7KzsrGztba3t7W1uLe6u7q4uL27vbm8wL29vMLEv8bCuaign56uwbrBwsTCxrOgnqSgvLGfoZugnrHJzM7eu9HOw6GbqaCcnp6fobTHv8DBvsDCvr+4vrm7uLi1t7i2tLKysq+ur7KqqrGoq6qvrq2urqynra6tpq2us6+xrrGysbCttbS0tLe2tbm6t7q6uri9vL28v7y/wcLAwMLBxL3BsaKhoKHBo6G7yMLDrpuprqeruLGgn56fo8nM0/HM1LKhmp2fnKSspJ+j6PvFw8O/wL+8vr6+vry4vLi6tLO0tLKzsa+urLCuqaytqq6vr7Cwsa+xr66qrrGwsrGws7Gxs7K0t7a4uLe3uLq9urm7u7y8vsC8wL/AwMXCwsDCwsLHtaqpoKbBtqejqq6oo623wqLJv6qinJyduMcVtdGgn5yZm5mipKqsn6LMysHDxMHEwMK+wL6/u7y5ure2ubW1tLCvsK+tra+uq62ur7GvsbGzsa2urqyss7GxsK6utbO1tLOzs7i1t7u6uLu6u7u8wb3BxMLCxcLCwcHGxcXDxL+3sa6poLOppKWsQJ6pc8fAp6nLxrSonZ2usLCvpZ6emp2gtaOhoKicqr/LxcTExMDCw73BwMO8vLi5ubW0tbOzsrCwrrKvsKqsrrGurbCvsK6zs7CwsamxsrCxsLKztbOytLa2ubm9ure4ure8vL6/vr/Fwr/GwsHAw8XCxqilssDCvLGqoZ+dtn+qoaGi7tPAW7vMf6mnnKfinqaempywsrK9pKOjq6cT38nIx8nExMXEwsPCvLy7vbe4tbO1tLKxs7Cvrq+xrrGwrrCysa+wtLOusLOxobO0tLOztbS3s7i3t7i3uLi8vLy6vr/AvMC/wsK/vMDEx8PHycrIs66fnaGsnrCkoJ+ksJ2jpaLAqLWvf8KoqqeyR6KcnZ6uqaaurJyboKPc08G6xMTHycfExcXDwL68u7y6uLi2sLeysrOws7CxsKyssLKzs7G1s66ys7KxsrGurbG0s7q2tbe3t7i5ubu+vrq8wL6/v73CwMPDxMbDxMfFxsfCx8XGtJ2omZ2en52npKimoLOunqKitqSopLSytbx/p5vNtKiqqqecnZ+fo6CgsMCuu8fMxMrFwcTCwMS+u7y6vLi1tbW3s7Swr7Cur7Gzs7G1tLWxsrG1s7WztLGjs7OtsLW4ubm5u7y8vLu+vsK+v7/Cxb3ExMPGx8fKx8fLyMrNy8zFp6Omqp+dobCtn5yzy7yhn5+eoaKgrLTPw6mgxrGbnqqpnpudnaGfoJ6xo524tcjLyMnGwb++w7y6vL66tri4t7W2s6+xsK6ur7C0tbO0s7Syt7Kzt7a1srayr7GwuLnAurm2vLy/wMG+wr7CwMbExcLFxcjKyczLycnIys7Pzcu5taexxLO+xr+nnq7LrqWen6ahoJ+qnqWoyaGqn56rp6qknqGlpZ+en6KYprDHy8nKyMjHwcTDwr6+u7u4tLi2tbextrGwsa+utrC0tLW0sraztrWzuLSztra4tbG0u76+vL6+u8DAvsDCxMHHx8bGx8bGy8bJycvKyM/Ozc3MxLq3pa+yxcPBzKefnrLRwqGho6Gmo8CfoK2in52ur7CboJ6dnp2en6CcwqGkrcvKzMnIycXEwsLBvsK9vrq1tLa1tbSysLGvsa21tra3tbW4srW4s7W3trq1urqxtby+vbzCvLu+wcHFxMa/xcPHx8nJy8rIyczKzsvPzc/T0qekorKln86v08jSz8umqTamoJ2qxMfDqaSlo9imo6Gmn5+fnaOspaCkn6CxnqbEzc/OzM/LxcXFwcPBvb27ubuzt7OztrKvs7Cur7e1ubeyu7a4ubS4tbe1t7u6urC4vLy9vL+7wMPCxMHDxcbJxMrIxcjLysrNysnLzdLSzLjLuKCcnrGmw7DWf7WuvbCjw6OjnqLAvn/HpWLcGKOhnp6an3+loqOqo52eoqW2x87Pzc3Oz8nFpLLCwsHDv7q6ura5trOzr7GxsbKutba6uLW6tbi6uLS6uby6uL28rrW/u728vMG+v8TBxcjJycbIysnMy8rNys7Q0NHQ0c/Foai3x6uhoqGipsDiqKOhoaanoK6ooMyuqpynrKefop2hnZyfoJ6qoLWhnKe4y8TP0M/N0M3Gq5uam6u8w8fAvbm9uLi4srWzsrSysrK6try4ubi1u7i3ub6+uLu8vLy3sL69wMLBxMTBxsbGycbJysvMzM/HzM7PzNHOz8/R0dKrqKGkvsKsn52dp8Seo75/4uC2BricrrGkpa+spaGf6Nmkq6ijnqe4n52k6H/Os83U09HKtZ6bnZ+crsbFwMS+ur23vLm4tbO0sq6xsbu6ub25ubu6vLq/ury9vb7Cu6GhtcDGwsPCycjDx8rJy8zJyczPzM3Q09TW0NPN0tPU0dTBtqegp6eeoJyfoaSkf7ror6S+qqTVsdfTf6efm6Gr1qqko56jpcKcn6XCRtnUqM3Or5+bm5yhrcDHycfCvLu6ur+8t7e2s7GusbKyur28ury8wLy7vrq/v76+wLqfmZ6uwsTExsjHyMjJxsrLzs7N0c/Pz9HP1tLS0tLV1dTW09XSuqumohSooJ6hnqOorbutpaGhqaq5ur6qn5+goamnoZ+no6y7sZ2foKqrs9O6uKSkmZubnrPEyMrGyb6+u8DAuLy4t7WxsrKwsLG8wL68v7y+v77Aw7zAwcHCuZ2bn6DByMbJysvKxsrLzM7PztPQ0tHP1tXU1dTZ1dfW1tbV2dezop2l1bejoJ6doZ6eoaSgoqCin6CgoKSenaG1p6KvqKCvsX/CnaCinqKj0sy0o6KdoaW8ttHMy8nHv8C+v7+5uLq4s7OzsrGur7/Av8G9v7y/wcLEwsTAxsC4np2cpsfJy8nLycvJzM3Nz9DO1NbSp6rP19fa2tnY2NnX2NjX2NDEraO+qqWjp6GcnZycnqGcnKGjraWfn6Oeo6Win6KgqqqorqOdoKCaoqbKsqSenpqgpa24zMnLxsTGwcC9uru4tra1s7GytbKwu7y7vr7AwMXAwcDEwcPBtqGioKCpxcvLycvMzc3Qy87S0dHS0r+ltqLBz9LY2Nrc2tva2d7a3tfTzKegnZ2fn52dn52fnpygn6mjoKGhpaGknqKin56gpKOgnqCgn5+enZ+Zl5ybm6GepZ6fpKPFxMHAwby6t7i4s7a0sbK0sLHAvr7CwMC9w7/BwsDFwL6mn6CgoaK1zMrJzszQ0dLP0M7R0dDVyce+pKKssc3a3NvZ3N7e3tzb3dymnp6doJudmaGcnJ6dm6Kjo6Omn6GgoaKhoZ6fnJ3gop+ZoZ2en5udmZ6emp0doqSbm5yioMfFxMHBv7q+urWztbGvrrGuq76+wsK+w8LHwsXBxcWqnqGhop6ho6PLzM3QztHR09LQ09XW1NfX2NbRqZ6hpLrO3dzc3NjP0Nrb2KOiuqakn6Ofn52dnpmcnKCdn6ajpaKfpaWip6Ghm6Obn52eoZ+bnZucmKCbnpye46Gmnq6kvcjHxca+u7a4urSxp5yjrq+wwb6/w8LEw8XAyMHDq6GjoaChpKGio6XOztPS0NHV1NbV1NjY19jX1r6rp6afnq+x1t3PpqCow9K9qrTJsKWhn5yjop2enp+mnaGlpKKkp6K4p+mfn5+cnJ6dnpudoKCZmZuZn5mbnJygn7zUyLKftMfCw8C7uLqzqpuXm6GtrK6/u7u+vMDBxMrJx7Gjo6Ghn6Ojo6Kkpq7Q09XX1dTU2NXZ1dva29nY2czCp6SloKOuxdSjo6OnzLCprqOgpKKfnqGfn52kn6OfxtCrpc7drH+noZ6enpyemp2enJ2kn5yenKCjobCunKH1sKadnZ6drqKjnZ2onqCfnJmYp6uusLq7vLy9wsHBxsi/paKioKOhpKWkpKSjpc3S1tPW1tXU19rZ29zb2d7e3t3Uq6ekoZ+hqaqjo6fcsKyrob2joLI2nqOcnp2cnqbLxKlEsaepqqWfoZ6cm52cnZybrK+tm52jpn+zvsGioZycmpydm52Ym5ubnJyamJugnqWrrK2uvcG/wL/AvsDAvqukpaWjpKakpKekpqSlqsnLzdHS0dTV09jZ2dnc3d7c0L+psaylpKSdoamhtMKypqmfoZ6fn6Ghn5ufn6SnqaejrNWgoKOnqJ62oaGfnZmdnJyosK6bnKGkqJ6eo5qbmJ2em56bn52bm5ydm5qZpKOnqa2tsLHFwcTFwMTFxMbCrrCzt7GuqqWlpqepp6mmx8nKys3P0tDW1NHU1NPT1tS7p6GkoKWkpaKhoKi73rSwqqCYnKGipKCjnaakpaeopKej4aOfpKijnp2gn5+enJ6boK2osLOzpp+ttZ2dn52cnJifnZyanJ2cm5qfoaSlp6ersK6wrca9wcTExcnOycrHzM/Lzs3Pzs/QzLuqq6q8ztLOz8/Rz87Q0dDQztHS1sjNrMavqaezsqWmnq3Wra2kpKaspavxpKGnrKGgoaemoaWiod6koqCdpJ6cn5yfnp6cpp6ytLW3saq7sZ2hnJycnJqempydnJ+np6qmpaalqausra2vwsXM3BPx8s3c/8XHztPQ09TR0tTS1NTTyLPp19fU0tLX09LT1dLX2NnV2MXWwOP123/Fs9Oon63Yqqax269EqsCxo5+uoaGhnqGgo6SioKahoKOjoJ2enaKeqKSitrO0trW+vuqzn52cm5ycmaCjo52krbObnJ6fo5mlsa6xsa3HzdHS1NLPx8rGwcfHvc3V2NfV2dfY1dfU1fsN4+LX19TV19bY09bY1dfX2LWlrq+qraimpLCqpa2qpq3XorPetamenKedoKKhoKOgnZ+enbfDp5uenZuenJygnLezsa6+ydUK3LOmpaGdnZ2cop6coa+4np6cm5iZmaeurrCzsBR/f380B/fq4tzY1MzV1trZ2NrU1NDU0c/a0dXZ3v53H/IE0tXb2rmwqrWwrqqnqKampaGmwbuouKajpKiwrKump6Kcnp+np6mepqKgoZ6bn6SdoJ2hoKCamaGbrMOwtcKy3dnUtKmfn6CfoZ6hp6GYm52ampuam6Sbq6+zsrCuf39/f39/f39/f39/1+R/f39/f04ZCPnx6ubf09jd397j6Of5DAsOadO6tq6srKilpaiqpqOqsKKuraSnvq4of3+moZ6emJyjn6WeoZ2iqaOdn52eoJ+jpJqdnp8QvqqloZ2mwrmtoMmpo6KooJugm5yam5ubm5icpKWvrLKwsq5/f39/f39/f39/f23c7H9/f39/f39/f39/f/DX3trb293c4OPi4uDevr7Ct7W1sqqnqKusx7aqseOzpKWopqbYrqemoaCgoJqfnaOen5+dnZ+fnaKjn6WirqGbn7yyuKOhnqCxt6yeo8C2v6Kbm5uXmJ+bmpyZnauurq+vsbGvrn9/f39/f39/f39/SeD5f39/f39/f39/f39/7+nf3Nve393g3OHf4uB/f39/f39/xKquOh+4ubi36q+lp6SloZ611aqmoqKioqGfoJyfoJ+gnKCeo6ChrZ6coJye07HWJZ+dnr2dn52dppibmpiYmZycnp2gmaWusK6usbC0s7Oxf39/f39/f39/f38y4QR/f39/f39/f39/f3/y6uHd2+Db3d/f3t/k5n9/f39Gf38RuLC0yb7Dtq42s6KkoaCjn6Kyv6iioKGgoaOgoJ+enJ2ioaGdoLOuqJ+dmpyr0Z2joKOin8OcnZuYmpuampubm5mborOnsrOxsbKvtrKytbN/f39/f39/f39/fx3gFX9/f39/f39/f39/f/Dn4eHf3eDe4N3c4uXpf39/5cbGfwGztQnHvt3Tv6inoqWhoqKgoaGhoaCkn6KmpaWppZ+cnaCfn6i8x7i2qqGcq6CevKuenbSdmJubnJyZnpiZoZ2eoaTAsrq1trC1sbS0s7Cyr39/f39/f39/f39/CeIif39/f39/f39/f39/8+bj4uDg4t3g4OLj5O1/f3/PzcnJOcq6Kb61r6q10aqko6OiqKefoqOjpKGdo6aiqqWjn6CfoKOjtbO6wMKxoqKeoKCfn6SeoJyamaG0nq2otaS8pbWmmZuhs7e1t7Wzs7iztK+vf39/f39/f39/f3/75TN/f39/f39/f39/f3//6+Lh4t/g4d3l4uPl7H9/f87izd9/TdW8ta6tq6mxuuyqo6KioaWgoqKjoKKdqKKhoaGloKGgoaHPubq5sKiioaahn5yjpp2bmZybnpujtMS3x8C3uZ6enpyls7aztrW1t7W2sLJ/f39/f39/f39/f/PiS39/f39/f39/f39/f+7m4eLg3d/f4uDj5uXwf39/F9j6f39/xLW2s6qtp6e00qesz8D0zqWipKaioqKio6uorqehoqOhpc2zsq6ppqakoJ+onJ2enZ+pqcLPrZ+cmJyfp7ikmp6jqLKitri1urW4tbWytH9/f39/f39/f39/7+Rif39/f39/f39/f39/6uPi4uHh3t/j4+Lk5O1/f39/f39/f3XEwr+5s66tq6upqqyrp6exzbCjs6SfpKKtqbfoqaSepKzLuLy4raWhocDEoKGqnKqlqsrf4+PUtq6qnJ+en6Wnnp2aoKe1s66wtbizs7O1Dx9Ef39/f39/f3/o4n9/f39/f39/f39/f3/p4+Lh4eHh4d/i5Obl939/f39/f39/YNHN39kBwrGwq6qqqqurqqenrqOlo6aopq2yx76oo6Stf7S038qto6e21aOcnqKfn6ehpLCooqyyvbido5+cnamcmpmdrJ6cnpyvsrG1sq9/f39/f39/f39/f+Thf39/f39iPTE2Rmh/aOzh4uHh4uXg4OTi5OX5f39/f39/f39E7X9/f39/5Liwq6ysqamtqbCmpKSqrKiosdDmqqWjoai5f3/Cs7G+saWknp6ho6CgnKKeoZydnp2fsry+o6aenKWhmJuhm5yfoLGyt7KvsH9/f39/f39/f39/5OR/f39/f39/f39/f3928uLj4eTf4N3f3+Hl4vx/f39/f39/fynyf39/f38HvK6vs7Gtr6uzs6yprqinsK7Cs626qqSjoaSvy9eq4n+wo6HCQaqsoaGmoqSjvKapt7GltKKfoKylpJydm5qfpKWssbWzsbKuf39/f39/f39/f3/h4n9/f39/f39/f39/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trCioq4MPiQfyK24saerzbDHs8LPpKCqpaOqp6SjpaytoqKeoqeko52enZ6mnp+jpeDEoKCgoZ+gnp6goJ6cnK6ZnZydnKefpKCdnKCfnKCmqqmmp6ipqKOfnJlUfzx/clCz5PX4+QIJBwYOFA4KCxMRB/v3z6Wwtqem1tinpaKgsaPa0gA94+X0qLemtt5/rrezr6ahpaa2p6KhoaOloqCjoaCkwqikp6GinqukoKeuq6uin6CmoZ+dnZyfoJ6bmZydnJ2vo52hp6ecnJudoaiqq6mlqqeoqKWenVp/LVn28PP5+/n09v4FBgIHBw4I/ffw7OWntMSz2a/Q16+qn66fy8ne/uWkqMyop6v3EbaoqK6yqKakpqQ1pKShoaSkoZ6ioqOptdbSoaGkp8yztX+npaSdoqGeop+foKSroJ6dmaCepLTdvKqlnZ+fnaKlp6eqqqepp6mnqaSmY+Ssqqmlp6ekpqimpaqoqaimpKSnq6impqqop6Smp6Wkq6Win6KkoKSuoq2lsL2lqLrTraqlsaujp6qopbGkpaKjoKx/pq6in65/5bWmrb7Pqq6obLakpKWkoaShpbefo6qhopubnZ+dsbXEvNuln5+cnZ+jpqanqqeopaOkp6RiVhReL2NhrKajop+mp3BppaOgo1d/saWko622qKWhpKWjr62oqaygpKihp6eg2aioI+Guf6qko6Ono6Gio6SupbynqaeenpuboLCko6OlpaKipqC587ifnqS8orCtq6SenpyinZ2pvp2euLG8qvSjnp2bm6Gio6GgmJuhoqSknGV5MHkxf2WzoqSfo6SpZW+npaOhs39nt6ejo7CvnaS4samztaeqt6SmpKSjn6qupq5Rr6mrqqqioZ+gnp+hoJ2sop+enqmfnpudn6WhoaCioaKio6q9zqihoKSos6x/vqCjqKKfnp2u26CuvZ+cnpmZnJmYnZ+Zm5mZmpiZmJ2YYGw3cSh3XOGioqGgo6g5Yaehnp2q2bjfqLymuKClpaSjpaazsaatp7u1n6Kdoaenqq7L8Km8qaWgnZ2bop+fn52dnJ2kyp2fnJ6eop+doZ+eoKGZpKl/qKKzo3/TsKehoJmvrrGfnp2dma+YnpiZl5icnJ2cnZqYmZeWmJiXm5dgajtpJWlG+6CdoJ+fo+U5uKCgoOC67rWspqOfoaSgop+jyjmspqenuMGgn56joqelxb+qun+qoZ+dpqGfnp2hnKWkpZ+enZucoaCenJ+nnJudnaCeoaatoamnq6mnnaOdm5ucoaCemZudm5eamZubm5mcrp2cm5qZlpaZl5qYl/X37vXu8uvJoZ6hoaCip8zOpqCgop+ipaegnaSdnqOio6sJsbmjpamepqGhop+hoqCoqMSkqqSfoKAiyaKcpp+hoJ6dm5qbmKWgnpygnaOenKCcmaCen56grn+q5sqrvaKkp6KhoZyan5+enZyZmJubmZmdpJ+Ym5mWl5mXmZaW', 'base64');
const depth = 1;

let frameData = Buffer.alloc(96 * 96 * 4);
let frameDataIx = 0;
for (let ix = 0; ix < snapshot.length; ix += depth) {
    if (depth === 1) {
        frameData[frameDataIx++] = snapshot[ix]; // r
        frameData[frameDataIx++] = snapshot[ix]; // g
        frameData[frameDataIx++] = snapshot[ix]; // b
        frameData[frameDataIx++] = 255;
    }
    else {
        frameData[frameDataIx++] = snapshot[ix + 0]; // r
        frameData[frameDataIx++] = snapshot[ix + 1]; // g
        frameData[frameDataIx++] = snapshot[ix + 2]; // b
        frameData[frameDataIx++] = 255;
    }
}

let jpegImageData = jpegjs.encode({
    data: frameData,
    width: 96,
    height: 96,
}, 100);

require('fs').writeFileSync('s.jpg', jpegImageData.data);