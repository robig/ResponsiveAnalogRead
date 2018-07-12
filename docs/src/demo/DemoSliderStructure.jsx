// @flow
import type {ComponentType} from 'react';
import type {Node} from 'react';
import type Parcel from 'parcels-react';

import React from 'react';
import {PureParcel} from 'parcels-react';
import Slider from 'rc-slider';
import 'rc-slider/assets/index.css';
import Structure from '../component/Structure';
import {Box, Text} from 'dcme-style';

type Props = {
    valueParcel: Parcel,
    height: number,
    label: string,
    min: number,
    max: number,
    layout?: ComponentType
};

type LayoutProps = {
    label: () => Node,
    slider: () => Node,
    value: () => Node
};

export default class DemoSliderStructure extends Structure<Props> {

    static elements = ['label', 'slider', 'value'];

    static layout = ({label, slider, value}: LayoutProps): Node => {
        return <Box>
            <Box modifier="marginBottomMilli">{slider()}</Box>
            {value()}
            {label()}
        </Box>;
    };

    label = (): Node => {
        let {label} = this.props;
        return <Text element="div" modifier="monospace center">{label}</Text>;
    };

    slider = (): Node => {
        let {
            valueParcel,
            height,
            min,
            max,
            ...other
        } = this.props;

        return <PureParcel parcel={valueParcel} forceUpdate={[min, max]}>
            {(parcel) => <Slider
                {...parcel.spread()}
                className="Slider"
                style={{height: height - 50}}
                vertical
                min={min}
                max={max}
                {...other}
            />}
        </PureParcel>;
    };

    value = (): Node => {
        let {valueParcel} = this.props;
        return <Text element="div" modifier="monospace center">{valueParcel.value()}</Text>;
    };
}