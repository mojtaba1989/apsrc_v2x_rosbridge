/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../J2735_201603.asn1"
 * 	`asn1c -S ../../skeletons -pdu=MessageFrame -fcompound-names -pdu=auto`
 */

#ifndef	_AnimalPropelledType_H_
#define	_AnimalPropelledType_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum AnimalPropelledType {
	AnimalPropelledType_unavailable	= 0,
	AnimalPropelledType_otherTypes	= 1,
	AnimalPropelledType_animalMounted	= 2,
	AnimalPropelledType_animalDrawnCarriage	= 3
	/*
	 * Enumeration is extensible
	 */
} e_AnimalPropelledType;

/* AnimalPropelledType */
typedef long	 AnimalPropelledType_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_AnimalPropelledType_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_AnimalPropelledType;
extern const asn_INTEGER_specifics_t asn_SPC_AnimalPropelledType_specs_1;
asn_struct_free_f AnimalPropelledType_free;
asn_struct_print_f AnimalPropelledType_print;
asn_constr_check_f AnimalPropelledType_constraint;
ber_type_decoder_f AnimalPropelledType_decode_ber;
der_type_encoder_f AnimalPropelledType_encode_der;
xer_type_decoder_f AnimalPropelledType_decode_xer;
xer_type_encoder_f AnimalPropelledType_encode_xer;
oer_type_decoder_f AnimalPropelledType_decode_oer;
oer_type_encoder_f AnimalPropelledType_encode_oer;
per_type_decoder_f AnimalPropelledType_decode_uper;
per_type_encoder_f AnimalPropelledType_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _AnimalPropelledType_H_ */
#include <asn_internal.h>
