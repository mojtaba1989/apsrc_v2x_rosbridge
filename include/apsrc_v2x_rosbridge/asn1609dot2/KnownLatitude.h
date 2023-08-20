/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IEEE1609dot2BaseTypes"
 * 	found in "../IEEE-1609.2-2016.asn1"
 * 	`asn1c -S ../../skeletons -pdu=Certificate -pdu=auto -pdu=Ieee1609Dot2Data -fcompound-names`
 */

#ifndef	_KnownLatitude_H_
#define	_KnownLatitude_H_


#include <asn_application.h>

/* Including external dependencies */
#include "NinetyDegreeInt.h"

#ifdef __cplusplus
extern "C" {
#endif

/* KnownLatitude */
typedef NinetyDegreeInt_t	 KnownLatitude_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_KnownLatitude;
asn_struct_free_f KnownLatitude_free;
asn_struct_print_f KnownLatitude_print;
asn_constr_check_f KnownLatitude_constraint;
ber_type_decoder_f KnownLatitude_decode_ber;
der_type_encoder_f KnownLatitude_encode_der;
xer_type_decoder_f KnownLatitude_decode_xer;
xer_type_encoder_f KnownLatitude_encode_xer;
oer_type_decoder_f KnownLatitude_decode_oer;
oer_type_encoder_f KnownLatitude_encode_oer;
per_type_decoder_f KnownLatitude_decode_uper;
per_type_encoder_f KnownLatitude_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _KnownLatitude_H_ */
#include <asn_internal.h>